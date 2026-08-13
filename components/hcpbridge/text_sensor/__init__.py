import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import CONF_TYPE

from .. import CONF_HCPBridge_ID, HCPBridge, hcpbridge_ns

DEPENDENCIES = ["hcpbridge"]

HCPBridgeTextSensor = hcpbridge_ns.class_(
    "HCPBridgeTextSensor", text_sensor.TextSensor, cg.Component
)

SensorType = hcpbridge_ns.enum("HCPBridgeTextSensorType")

TYPES = {
    "state": SensorType.HCPBRIDGE_TEXT_STATE,
    "serial_number": SensorType.HCPBRIDGE_TEXT_SERIAL_NUMBER,
    "firmware_version": SensorType.HCPBRIDGE_TEXT_FIRMWARE_VERSION,
    # Why the link is down, not just that it is: a bus nothing has ever
    # arrived on needs different attention than one that fell quiet.
    "link_state": SensorType.HCPBRIDGE_TEXT_LINK,
}

CONFIG_SCHEMA = (
    text_sensor.text_sensor_schema(HCPBridgeTextSensor)
    .extend(
        {
            cv.GenerateID(CONF_HCPBridge_ID): cv.use_id(HCPBridge),
            cv.Optional(CONF_TYPE, default="state"): cv.enum(TYPES, lower=True),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await text_sensor.new_text_sensor(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_HCPBridge_ID])
    cg.add(var.set_hcpbridge_parent(parent))
    cg.add(var.set_sensor_type(config[CONF_TYPE]))
