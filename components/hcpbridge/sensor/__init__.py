import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_TYPE,
    ICON_PERCENT,
    STATE_CLASS_MEASUREMENT,
    UNIT_PERCENT,
)

from .. import CONF_HCPBridge_ID, HCPBridge, hcpbridge_ns

DEPENDENCIES = ["hcpbridge"]

HCPBridgeSensor = hcpbridge_ns.class_("HCPBridgeSensor", sensor.Sensor, cg.Component)

# Which position to report. Defaults to the current one, so configurations
# written before this option keep working unchanged.
TYPE_POSITION = "position"
TYPE_TARGET_POSITION = "target_position"
TYPES = [TYPE_POSITION, TYPE_TARGET_POSITION]

CONFIG_SCHEMA = (
    sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT,
        icon=ICON_PERCENT,
        state_class=STATE_CLASS_MEASUREMENT,
        accuracy_decimals=0,
    )
    .extend({cv.GenerateID(): cv.declare_id(HCPBridgeSensor)})
    .extend(
        {cv.Optional(CONF_TYPE, default=TYPE_POSITION): cv.one_of(*TYPES, lower=True)}
    )
    .extend({cv.GenerateID(CONF_HCPBridge_ID): cv.use_id(HCPBridge)})
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    parent = await cg.get_variable(config[CONF_HCPBridge_ID])
    cg.add(var.set_hcpbridge_parent(parent))
    cg.add(var.set_target_mode(config[CONF_TYPE] == TYPE_TARGET_POSITION))
    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)
