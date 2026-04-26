import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import output
from esphome.const import CONF_ID, CONF_METHOD, CONF_MIN_POWER

CODEOWNERS = ["@victorvation"]

DEPENDENCIES = ["output"]

advanced_ac_dimmer_ns = cg.esphome_ns.namespace("advanced_ac_dimmer")
AcDimmer = advanced_ac_dimmer_ns.class_("AcDimmer", output.FloatOutput, cg.Component)

DimMethod = advanced_ac_dimmer_ns.enum("DimMethod")
ZcMethod = advanced_ac_dimmer_ns.enum("ZcMethod")

# Dimming methods — controls how the gate pin is driven relative to zero crossing:
#   leading_pulse : brief gate pulse at start of half-cycle. TRIAC only. Default.
#   leading       : gate held high from start of half-cycle. TRIAC only.
#   trailing      : gate held high from zero cross until dim point. Back-to-back MOSFET only.
DIM_METHODS = {
    "leading_pulse": DimMethod.DIM_METHOD_LEADING_PULSE,
    "leading": DimMethod.DIM_METHOD_LEADING,
    "trailing": DimMethod.DIM_METHOD_TRAILING,
}

# Zero crossing detection methods — select to match your ZCD circuit:
#   edges          : INTERRUPT_ANY_EDGE. Sustained-level circuit (e.g. H11A1-based:
#                    0V on positive half-cycle, 3.3V on negative half-cycle).
#                    Both rising and falling edges represent zero crossings. Default.
#   pulse          : INTERRUPT_FALLING_EDGE. Active-low narrow pulse at each ZC.
#                    Equivalent to the original upstream ac_dimmer behaviour.
#   inverted_pulse : INTERRUPT_RISING_EDGE. Active-high narrow pulse at each ZC.
ZC_METHODS = {
    "edges": ZcMethod.ZC_METHOD_EDGES,
    "pulse": ZcMethod.ZC_METHOD_PULSE,
    "inverted_pulse": ZcMethod.ZC_METHOD_INVERTED_PULSE,
}

CONF_ZERO_CROSS_PIN = "zero_cross_pin"
CONF_GATE_PIN = "gate_pin"
CONF_INIT_WITH_N_HALF_CYCLES = "init_with_n_half_cycles"
CONF_MAX_FLAT_THRESHOLD = "max_flat_threshold"
CONF_RMS_CORRECTION = "rms_correction"
CONF_ZC_METHOD = "zc_method"


def validate_flat_zone(config):
    if CONF_MAX_FLAT_THRESHOLD in config:
        threshold = config[CONF_MAX_FLAT_THRESHOLD]
        max_power = config.get("max_power", 1.0)
        if max_power < threshold:
            raise cv.Invalid(
                f"'max_power' ({max_power}) must be >= 'max_flat_threshold' ({threshold}): "
                "the jump target cannot be lower than the threshold that triggers it"
            )
    return config


CONFIG_SCHEMA = cv.All(
    output.FLOAT_OUTPUT_SCHEMA.extend(
        {
            cv.Required(CONF_ID): cv.declare_id(AcDimmer),
            cv.Required(CONF_GATE_PIN): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_ZERO_CROSS_PIN): pins.internal_gpio_input_pin_schema,

            # Dimming method — select based on your power switch type.
            cv.Optional(CONF_METHOD, default="leading_pulse"): cv.enum(
                DIM_METHODS, lower=True
            ),

            # Zero crossing detection method — select based on your ZCD circuit.
            # edges (default) covers H11A1-based sustained-level circuits.
            cv.Optional(CONF_ZC_METHOD, default="edges"): cv.enum(
                ZC_METHODS, lower=True
            ),

            # Kickstart: number of full half-cycles at full conduction on turn-on.
            # Helps LED drivers that need sustained power before they will start.
            # 0 = disabled (default). 1 = equivalent to upstream init_with_half_cycle: true.
            cv.Optional(CONF_INIT_WITH_N_HALF_CYCLES, default=0): cv.int_range(
                min=0, max=255
            ),

            # RMS power compensation via acos transform.
            # true (default): correct for incandescent/resistive loads (brightness ∝ RMS power).
            # false: better for LED lamps with constant-current drivers (brightness ∝ conduction
            #        fraction, which is already linear — acos over-corrects for these loads).
            cv.Optional(CONF_RMS_CORRECTION, default=True): cv.boolean,

            # Flat zone threshold (0.01–0.99). When set, the dimmable range is compressed
            # into slider positions 1–99%, and slider 100% jumps directly to max_power.
            # Eliminates the dead zone where the MOSFET switches but lamp output is
            # perceptually identical to maximum, reducing heat and switching losses.
            # Requires gamma_correct: 0 (or 1) on the light entity — see README.
            cv.Optional(CONF_MAX_FLAT_THRESHOLD): cv.float_range(min=0.01, max=0.99),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    validate_flat_zone,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await output.register_output(var, config)

    gate_pin = await cg.gpio_pin_expression(config[CONF_GATE_PIN])
    cg.add(var.set_gate_pin(gate_pin))

    zero_cross_pin = await cg.gpio_pin_expression(config[CONF_ZERO_CROSS_PIN])
    cg.add(var.set_zero_cross_pin(zero_cross_pin))

    cg.add(var.set_method(config[CONF_METHOD]))
    cg.add(var.set_zc_method(config[CONF_ZC_METHOD]))
    cg.add(var.set_init_with_n_half_cycles(config[CONF_INIT_WITH_N_HALF_CYCLES]))
    cg.add(var.set_rms_correction(config[CONF_RMS_CORRECTION]))

    if CONF_MAX_FLAT_THRESHOLD in config:
        cg.add(var.set_max_flat_threshold(config[CONF_MAX_FLAT_THRESHOLD]))
