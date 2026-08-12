from esphome.components import text_sensor
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_TYPE
from .. import hcpbridge_ns, CONF_HCPBridge_ID, HCPBridge

DEPENDENCIES = ["hcpbridge"]

HCPBridgeTextSensor = hcpbridge_ns.class_("HCPBridgeTextSensor", text_sensor.TextSensor, cg.Component)

SensorType = hcpbridge_ns.enum("HCPBridgeTextSensorType")

TYPES = {
    "state": SensorType.HCPBRIDGE_TEXT_STATE,
    "serial_number": SensorType.HCPBRIDGE_TEXT_SERIAL_NUMBER,
    "firmware_version": SensorType.HCPBRIDGE_TEXT_FIRMWARE_VERSION,
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
