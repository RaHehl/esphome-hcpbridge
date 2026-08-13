from esphome import pins
import esphome.codegen as cg
from esphome.components.esp32 import (
    FRAMEWORK_ESP_IDF,
    VARIANT_ESP32,
    VARIANT_ESP32S3,
    get_esp32_variant,
)
import esphome.config_validation as cv
from esphome.const import (
    CONF_BAUD_RATE,
    CONF_ESPHOME,
    CONF_HARDWARE_UART,
    CONF_ID,
    CONF_LIBRARIES,
    CONF_LOGGER,
    CONF_NAME,
    CONF_RX_PIN,
    CONF_TX_PIN,
)
import esphome.final_validate as fv

CONF_RTS_PIN = "rts_pin"
CONF_UART_NUM = "uart_num"
CONF_PROTOCOL = "protocol"

# Which bus this drive speaks, per door rather than per image, so one board can
# serve two doors of different generations. A Series 4 and up speaks hcp2; a
# SupraMatic E3 speaks hcp1.
PROTOCOL_HCP2 = "hcp2"
PROTOCOL_HCP1 = "hcp1"
PROTOCOLS = [PROTOCOL_HCP2, PROTOCOL_HCP1]

# Entity types the older bus has no way of feeding. Refused rather than offered
# and left reading nothing for ever.
HCP1_ABSENT = {
    ("sensor", "position"): "the position; this bus reports open and shut only",
    ("sensor", "target_position"): "a target position",
    ("text_sensor", "serial_number"): "the drive's serial number",
    ("text_sensor", "firmware_version"): "the drive's firmware version",
    ("switch", "half"): "the half-open position",
    ("button", "half"): "the half-open position",
}

hcpbridge_ns = cg.esphome_ns.namespace("hcpbridge")
HCPBridge = hcpbridge_ns.class_("HCPBridge", cg.PollingComponent)

CONF_HCPBridge_ID = "hcpbridge_id"

# A board with a spare UART can serve a second door. A single block stays valid,
# so nothing written before this needs changing.
MULTI_CONF = True

# Full (high-power) UARTs per variant, from the SoC's SOC_UART_HP_NUM.
#
# Deliberately not SOC_UART_NUM: on the C5, C6 and P4 that number also counts a
# low-power UART, whose 16-byte FIFO is a fraction of the 128 bytes a frame on
# this bus can need. A configuration naming it would validate and then starve.
HP_UART_COUNT = {
    "ESP32": 3,
    "ESP32C2": 2,
    "ESP32C3": 2,
    "ESP32C5": 2,
    "ESP32C6": 2,
    "ESP32C61": 3,
    "ESP32H2": 2,
    "ESP32H4": 2,
    "ESP32H21": 2,
    "ESP32P4": 5,
    "ESP32S2": 2,
    "ESP32S3": 3,
    "ESP32S31": 4,
}

# The table is checked against ESPHome's variant list by test/unit/variant_table.py
# rather than here: an import-time assert would stop the component loading for
# everyone the day ESPHome learns a new chip, including people who will never
# own one.

# Only these two carry pin defaults that were ever tried against a drive.
# Elsewhere the wiring has to be stated: a guessed pair that happens to validate
# is worse than one that refuses to, because the drive answers neither.
#
# Filled in here rather than left to the C++ side, so that the pins this bus
# will actually use show up in `esphome config` and can be checked against the
# rest of the configuration below.
PIN_DEFAULTS = {
    VARIANT_ESP32: {CONF_RX_PIN: 16, CONF_TX_PIN: 17},
    VARIANT_ESP32S3: {CONF_RX_PIN: 18, CONF_TX_PIN: 17},
}

# The ROM loader prints here at every boot, so it is never handed out, not even
# when the logger has left it free.
CONSOLE_UART = 0


def _validate_variant(config):
    # Schema extraction passes a sentinel rather than a configuration; touching
    # it as a dict breaks the dashboard's schema dump.
    if config is cv.SCHEMA_EXTRACT:
        return config

    variant = get_esp32_variant()
    if variant not in HP_UART_COUNT:
        raise cv.Invalid(
            f"hcpbridge does not know how many UARTs the {variant} has, so it "
            "cannot tell a usable port from a low-power one. This is a gap in "
            "the component, not in your configuration; please report the chip."
        )
    hp_uarts = HP_UART_COUNT[variant]
    config = dict(config)

    if CONF_UART_NUM not in config:
        # UART2 wherever it exists, which leaves every configuration written so
        # far on the port it already used; UART1 on the two-UART variants.
        config[CONF_UART_NUM] = 2 if hp_uarts > 2 else 1

    uart_num = config[CONF_UART_NUM]
    if uart_num >= hp_uarts:
        raise cv.Invalid(
            f"the {variant} has {hp_uarts} full UARTs, so uart_num has to be "
            f"0..{hp_uarts - 1}. Low-power UARTs are not counted: their FIFO is "
            "too small for this bus.",
            path=[CONF_UART_NUM],
        )
    if uart_num == CONSOLE_UART:
        raise cv.Invalid(
            "uart_num 0 is the console: the ROM loader prints on it at every "
            "boot, and the drive would read that as a malformed frame.",
            path=[CONF_UART_NUM],
        )

    defaults = PIN_DEFAULTS.get(variant)
    if defaults is None:
        missing = [k for k in (CONF_RX_PIN, CONF_TX_PIN) if k not in config]
        if missing:
            raise cv.Invalid(
                f"{' and '.join(missing)} are required on the {variant}: only "
                "the ESP32 and the ESP32-S3 have defaults that were tested "
                "against a drive.",
                path=[missing[0]],
            )
    else:
        for key, pin in defaults.items():
            config.setdefault(key, pin)
    return config


def _logger_uart(full_config):
    """Which hardware UART the logger occupies, or None if it occupies none."""
    logger_conf = full_config.get(CONF_LOGGER)
    if logger_conf is None:
        return None
    # Baud rate 0 turns the serial console off and gives the port back.
    if logger_conf.get(CONF_BAUD_RATE) == 0:
        return None
    hw_uart = logger_conf.get(CONF_HARDWARE_UART)
    # USB_CDC and USB_SERIAL_JTAG are not UARTs, and are the default on every
    # variant that has USB.
    if not isinstance(hw_uart, str) or not hw_uart.startswith("UART"):
        return None
    try:
        return int(hw_uart[len("UART") :])
    except ValueError:
        return None


def _other_instances(full_config, config):
    ours = full_config.get("hcpbridge")
    if ours is None:
        return []
    if isinstance(ours, dict):
        ours = [ours]
    return [c for c in ours if c.get(CONF_ID) != config.get(CONF_ID)]


def _final_validate(config):
    full_config = fv.full_config.get()
    variant = get_esp32_variant()
    uart_num = config[CONF_UART_NUM]

    # The replaced library is Arduino-only. Left in an esp-idf configuration it
    # validates, then fails the build after the whole toolchain has been
    # fetched and a thousand objects compiled, naming Arduino.h rather than the
    # migration step that was missed.
    for lib in (full_config.get(CONF_ESPHOME) or {}).get(CONF_LIBRARIES, []):
        if "modbus-esp8266" in str(lib):
            raise cv.Invalid(
                f"the library '{lib}' cannot be built under esp-idf, and this "
                "component no longer needs it: it brings its own Modbus RTU "
                "server. Remove the esphome: libraries: entry."
            )

    # Two buses on one port would each read half of the other's frames, and each
    # would look to its drive like an accessory that answers erratically.
    for other in _other_instances(full_config, config):
        if other.get(CONF_UART_NUM) == uart_num:
            raise cv.Invalid(
                f"uart_num {uart_num} is used by more than one hcpbridge. Each "
                "door needs its own port.",
                path=[CONF_UART_NUM],
            )
        for key in (CONF_RX_PIN, CONF_TX_PIN):
            if key in config and other.get(key) == config[key]:
                raise cv.Invalid(
                    f"GPIO{config[key]} is wired to more than one hcpbridge.",
                    path=[key],
                )

    if config[CONF_PROTOCOL] == PROTOCOL_HCP1:
        for (domain, entity_type), what in HCP1_ABSENT.items():
            for entry in full_config.get(domain) or []:
                if entry.get("platform") != "hcpbridge":
                    continue
                if entry.get(CONF_HCPBridge_ID) != config.get(CONF_ID):
                    continue
                if entry.get("type") == entity_type:
                    # Named, because validation has already filled in defaults:
                    # a sensor without a type: line reads as position here, so
                    # the type the message asks for may not be in the file.
                    who = entry.get(CONF_NAME) or entry.get(CONF_ID)
                    which = f" '{who}'" if who else ""
                    raise cv.Invalid(
                        f"this drive speaks hcp1, which does not carry {what}. "
                        f"Remove the {domain}{which}; its type is {entity_type}, "
                        "which is also what a missing type: line means."
                    )

    logger_uart = _logger_uart(full_config)
    if logger_uart == uart_num:
        raise cv.Invalid(
            f"uart_num {uart_num} is the one the logger writes to. Give this "
            "component another port, or set logger: baud_rate: 0 - its output "
            "otherwise lands between a frame and the answer the drive waits for."
        )

    # The uart: platform hands out ports at runtime, counting up from 0 and
    # stepping over the logger's. We install our port ourselves, so an overlap
    # is only visible from here.
    uart_conf = full_config.get("uart")
    if uart_conf:
        taken, port = set(), 0
        for _ in range(len(uart_conf)):
            if port == logger_uart:
                port += 1
            taken.add(port)
            port += 1
        if uart_num in taken:
            raise cv.Invalid(
                f"uart_num {uart_num} is also claimed by a uart: component. "
                f"With this configuration those take {sorted(taken)}; pick a "
                "port outside that range."
            )

    # On a WROVER these two are the PSRAM lines. ESPHome does not reject them,
    # so the build succeeds and then either PSRAM or this bus is dead.
    if variant == VARIANT_ESP32 and "psram" in full_config:
        for key in (CONF_RX_PIN, CONF_TX_PIN):
            if config.get(key) in (16, 17):
                raise cv.Invalid(
                    f"GPIO{config[key]} carries PSRAM on WROVER modules, and "
                    "psram: is enabled. Wire this bus to other pins.",
                    path=[key],
                )
    return config


# The sources use the ESP-IDF UART driver and FreeRTOS directly, so the
# framework is not negotiable. Everything else is checked above rather than
# left to fail inside the compiler.
CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(HCPBridge),
            # Numbers, not pin schemas: only the number is ever used, and
            # accepting inverted/mode/drive_strength would silently drop them.
            cv.Optional(CONF_RX_PIN): pins.internal_gpio_input_pin_number,
            cv.Optional(CONF_TX_PIN): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_RTS_PIN): pins.internal_gpio_output_pin_number,
            # The variant decides both the default and the upper bound; see
            # _validate_variant.
            cv.Optional(CONF_UART_NUM): cv.int_range(
                min=0, max=max(HP_UART_COUNT.values()) - 1
            ),
            cv.Optional(CONF_PROTOCOL, default=PROTOCOL_HCP2): cv.one_of(
                *PROTOCOLS, lower=True
            ),
        }
    ).extend(cv.polling_component_schema("500ms")),
    cv.only_on_esp32,
    cv.only_with_framework(FRAMEWORK_ESP_IDF),
    _validate_variant,
)

FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_RX_PIN in config:
        cg.add(var.set_rx_pin(config[CONF_RX_PIN]))
    if CONF_TX_PIN in config:
        cg.add(var.set_tx_pin(config[CONF_TX_PIN]))
    if CONF_RTS_PIN in config:
        cg.add(var.set_rts_pin(config[CONF_RTS_PIN]))
    cg.add(var.set_uart_num(config[CONF_UART_NUM]))
    # Per instance, so one board can serve doors of different generations.
    cg.add(var.set_hcp1(config[CONF_PROTOCOL] == PROTOCOL_HCP1))
