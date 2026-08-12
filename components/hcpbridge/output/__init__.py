from esphome.components import output

import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_ID
from .. import hcpbridge_ns, HCPBridge, CONF_HCPBridge_ID

DEPENDENCIES = ["hcpbridge"]

HCPBridgeBinaryOutput = hcpbridge_ns.class_("HCPBridgeBinaryOutput", output.BinaryOutput, cg.Component)

CONFIG_SCHEMA = output.BINARY_OUTPUT_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(HCPBridgeBinaryOutput),
        cv.GenerateID(CONF_HCPBridge_ID): cv.use_id(HCPBridge),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await output.register_output(var, config)

    parent = await cg.get_variable(config[CONF_HCPBridge_ID])
    cg.add(var.set_hcpbridge_parent(parent))
