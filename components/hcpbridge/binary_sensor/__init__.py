import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_TYPE,
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_PROBLEM,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

from .. import CONF_HCPBridge_ID, HCPBridge, hcpbridge_ns

DEPENDENCIES = ["hcpbridge"]

HCPBridgeBinarySensor = hcpbridge_ns.class_(
    "HCPBridgeBinarySensor", binary_sensor.BinarySensor, cg.Component
)
SensorType = hcpbridge_ns.enum("HCPBridgeBinarySensorType")

TYPES = {
    "is_connected": SensorType.HCPBRIDGE_BINARY_IS_CONNECTED,
    "relay_state": SensorType.HCPBRIDGE_BINARY_RELAY_STATE,
    "actuator_error": SensorType.HCPBRIDGE_BINARY_ACTUATOR_ERROR,
}

# Device class and category belong to what is being reported, not to whoever
# writes the configuration, so they follow the type rather than being repeated
# in every YAML.
DEFAULTS = {
    "is_connected": {"device_class": DEVICE_CLASS_CONNECTIVITY},
    "actuator_error": {
        "device_class": DEVICE_CLASS_PROBLEM,
        "entity_category": ENTITY_CATEGORY_DIAGNOSTIC,
    },
}

CONFIG_SCHEMA = cv.typed_schema(
    {
        name: binary_sensor.binary_sensor_schema(
            HCPBridgeBinarySensor, **DEFAULTS.get(name, {})
        )
        .extend({cv.GenerateID(CONF_HCPBridge_ID): cv.use_id(HCPBridge)})
        .extend(cv.COMPONENT_SCHEMA)
        for name in TYPES
    },
    key=CONF_TYPE,
)


async def to_code(config):
    var = await binary_sensor.new_binary_sensor(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_HCPBridge_ID])
    cg.add(var.set_hcpbridge_parent(parent))
    cg.add(var.set_sensor_type(TYPES[config[CONF_TYPE]]))
