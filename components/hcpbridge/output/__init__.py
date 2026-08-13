import esphome.codegen as cg
from esphome.components import output
import esphome.config_validation as cv
from esphome.const import CONF_ID

from .. import CONF_HCPBridge_ID, HCPBridge, hcpbridge_ns

DEPENDENCIES = ["hcpbridge"]

HCPBridgeBinaryOutput = hcpbridge_ns.class_(
    "HCPBridgeBinaryOutput", output.BinaryOutput, cg.Component
)

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
