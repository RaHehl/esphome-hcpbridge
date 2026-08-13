import esphome.codegen as cg
from esphome.components import switch
import esphome.config_validation as cv
from esphome.const import CONF_TYPE

from .. import CONF_HCPBridge_ID, HCPBridge, hcpbridge_ns

DEPENDENCIES = ["hcpbridge"]

HCPBridgeSwitch = hcpbridge_ns.class_("HCPBridgeSwitch", switch.Switch, cg.Component)
SwitchType = hcpbridge_ns.enum("HCPBridgeSwitchType")

TYPES = {
    "vent": SwitchType.HCPBRIDGE_SWITCH_VENT,
    "half": SwitchType.HCPBRIDGE_SWITCH_HALF,
}

ICONS = {
    "vent": "mdi:hvac",
    "half": "mdi:fraction-one-half",
}

CONFIG_SCHEMA = cv.typed_schema(
    {
        name: switch.switch_schema(HCPBridgeSwitch, icon=ICONS[name])
        .extend({cv.GenerateID(CONF_HCPBridge_ID): cv.use_id(HCPBridge)})
        .extend(cv.COMPONENT_SCHEMA)
        for name in TYPES
    },
    key=CONF_TYPE,
)


async def to_code(config):
    var = await switch.new_switch(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_HCPBridge_ID])
    cg.add(var.set_hcpbridge_parent(parent))
    cg.add(var.set_switch_type(TYPES[config[CONF_TYPE]]))
