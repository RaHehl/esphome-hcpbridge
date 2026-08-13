import esphome.codegen as cg
from esphome.components import button
import esphome.config_validation as cv
from esphome.const import CONF_TYPE, ICON_FAN

from .. import CONF_HCPBridge_ID, HCPBridge, hcpbridge_ns

DEPENDENCIES = ["hcpbridge"]

HCPBridgeButton = hcpbridge_ns.class_("HCPBridgeButton", button.Button, cg.Component)
ButtonType = hcpbridge_ns.enum("HCPBridgeButtonType")

TYPES = {
    "impulse": ButtonType.HCPBRIDGE_BUTTON_IMPULSE,
    "vent": ButtonType.HCPBRIDGE_BUTTON_VENT,
    "half": ButtonType.HCPBRIDGE_BUTTON_HALF,
}

ICONS = {
    "impulse": "mdi:arrow-up-down",
    "vent": ICON_FAN,
    "half": "mdi:fraction-one-half",
}

CONFIG_SCHEMA = cv.typed_schema(
    {
        name: button.button_schema(HCPBridgeButton, icon=ICONS[name])
        .extend({cv.GenerateID(CONF_HCPBridge_ID): cv.use_id(HCPBridge)})
        .extend(cv.COMPONENT_SCHEMA)
        for name in TYPES
    },
    key=CONF_TYPE,
)


async def to_code(config):
    var = await button.new_button(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_HCPBridge_ID])
    cg.add(var.set_hcpbridge_parent(parent))
    cg.add(var.set_button_type(TYPES[config[CONF_TYPE]]))
