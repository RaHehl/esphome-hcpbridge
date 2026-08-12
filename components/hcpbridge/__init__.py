import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components.esp32 import (
    FRAMEWORK_ESP_IDF,
    VARIANT_ESP32,
    VARIANT_ESP32S3,
    only_on_variant,
)
from esphome.const import (
    CONF_ID,
    CONF_RX_PIN,
    CONF_TX_PIN,
)

CONF_RTS_PIN = "rts_pin"
CONF_UART_NUM = "uart_num"

hcpbridge_ns = cg.esphome_ns.namespace("hcpbridge")
HCPBridge = hcpbridge_ns.class_("HCPBridge", cg.PollingComponent)

CONF_HCPBridge_ID = "hcpbridge_id"

# The sources use the ESP-IDF UART driver and FreeRTOS directly, and the default
# port is UART2, which only the variants below have. Without this the config
# validates and then fails inside the compiler, which is the failure this
# prevents.
CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(HCPBridge),
            # Numbers, not pin schemas: only the number is ever used, and
            # accepting inverted/mode/drive_strength would silently drop them.
            cv.Optional(CONF_RX_PIN): pins.internal_gpio_input_pin_number,
            cv.Optional(CONF_TX_PIN): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_RTS_PIN): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_UART_NUM, default=2): cv.int_range(min=0, max=2),
        }
    ).extend(cv.polling_component_schema("500ms")),
    cv.only_on_esp32,
    cv.only_with_framework(FRAMEWORK_ESP_IDF),
    only_on_variant(supported=[VARIANT_ESP32, VARIANT_ESP32S3]),
)


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
