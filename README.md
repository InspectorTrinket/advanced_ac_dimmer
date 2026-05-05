# Advanced AC Dimmer

An ESPHome external component for AC phase-angle dimming, extending the built-in `ac_dimmer`
component with several features needed for real-world installations.

## Features vs upstream `ac_dimmer`

| Feature | upstream | advanced_ac_dimmer |
|---|---|---|
| Zero crossing detection | falling edge only | **edges / pulse / inverted_pulse** |
| Multi-half-cycle kickstart | single half-cycle | **configurable count** |
| RMS compensation control | always on | **enable/disable per load type** |
| Flat zone / dead zone elimination | no | **max_flat_threshold** |
| Half-cycle offset (MOSFET mismatch compensation) | no | **half_cycle_offset** |
| Trailing method (back-to-back MOSFET) | yes | yes |
| Leading / leading pulse (TRIAC) | yes | yes |
| Multi-channel shared ZC pin | yes | yes |
| ESP32 / ESP8266 | yes | yes |

---

## Installation

Add to your ESPHome YAML:

```yaml
external_components:
  - source: github://YOUR_GITHUB_USERNAME/advanced_ac_dimmer
    components: [advanced_ac_dimmer]
```

---

## Full configuration example

```yaml
output:
  - platform: advanced_ac_dimmer
    id: dimmer1
    gate_pin: GPIO05             # Required. GPIO connected to gate optocoupler input.
    zero_cross_pin:              # Required. GPIO connected to ZCD circuit output.
      number: GPIO03
      allow_other_uses: true     # Required when sharing ZC pin between channels.
      mode:
        input: true
        pullup: false
      inverted: false
    method: trailing             # trailing | leading_pulse | leading
    zc_method: edges             # edges | pulse | inverted_pulse
    init_with_n_half_cycles: 10  # 0–255, default 0
    rms_correction: false        # true | false, default true
    min_power: 0.01              # 0.0–1.0, default 0.0
    max_power: 1.0               # 0.0–1.0, default 1.0
    max_flat_threshold: 0.63     # 0.01–0.99, optional
    half_cycle_offset: 0         # -500 to +500 µs, default 0

light:
  - platform: monochromatic
    name: "Living Room"
    output: dimmer1
    gamma_correct: 0             # Required when using max_flat_threshold. See below.
    restore_mode: ALWAYS_OFF
    default_transition_length: 1s
```

### Two channels sharing one ZC pin

```yaml
output:
  - platform: advanced_ac_dimmer
    id: dimmer1
    gate_pin: GPIO05
    zero_cross_pin:
      number: GPIO03
      allow_other_uses: true
      mode: { input: true, pullup: false }
    method: trailing
    zc_method: edges

  - platform: advanced_ac_dimmer
    id: dimmer2
    gate_pin: GPIO04
    zero_cross_pin:
      number: GPIO03             # Same pin — allow_other_uses handles this internally.
      allow_other_uses: true
      mode: { input: true, pullup: false }
    method: trailing
    zc_method: edges
```

---

## Option reference

### `method`
Controls how the gate pin is driven relative to each zero crossing.

| Value | Description | Use with |
|---|---|---|
| `leading_pulse` | Brief gate pulse at start of half-cycle | TRIAC (default) |
| `leading` | Gate held high from start of half-cycle | TRIAC |
| `trailing` | Gate held high from zero cross until dim point | Back-to-back MOSFET |

---

### `zc_method`
Selects the GPIO interrupt mode to match your ZCD circuit output.

| Value | Interrupt | Circuit type |
|---|---|---|
| `edges` | `ANY_EDGE` | Sustained-level circuit — 0V on positive half-cycle, 3.3V on negative (e.g. NPN transistor or H11A1-based). **Default.** |
| `pulse` | `FALLING_EDGE` | Active-low narrow pulse at each zero crossing. Equivalent to upstream `ac_dimmer`. |
| `inverted_pulse` | `RISING_EDGE` | Active-high narrow pulse at each zero crossing. |

**Why this matters:** The original `ac_dimmer` uses `FALLING_EDGE` only, detecting one zero
crossing per mains cycle instead of two. With a sustained-level ZCD circuit, this causes the
dimmer to fire on one half-cycle and be dark on the other — visible flicker and half the
expected output power. Setting `zc_method: edges` fixes this without any hardware changes.

---

### `init_with_n_half_cycles`
Number of full half-cycles at maximum conduction sent when the output turns on from off.
Provides sustained power to help LED drivers whose internal capacitors need time to charge
before the LED will conduct. `0` disables the feature (default). `1` is equivalent to the
upstream `init_with_half_cycle: true`.

At 60Hz each half-cycle is 8.33ms, so `init_with_n_half_cycles: 12` ≈ 100ms of full power.
Tune by increasing until the lamp reliably ignites on every turn-on.

---

### `rms_correction`
When `true` (default), applies an `acos` transform to compensate for the nonlinear
relationship between phase-angle conduction fraction and RMS power delivered to the load.
This is correct for **resistive / incandescent loads** where brightness is proportional to
RMS power.

For **LED lamps with constant-current switching drivers**, brightness is proportional to
conduction fraction (already linear). The `acos` transform over-corrects and makes dimming
less accurate. Set `rms_correction: false` for LED loads.

---

### `max_flat_threshold`
Eliminates the flat zone — the region of the dimming range where the lamp output is
perceptually indistinguishable from maximum, but the MOSFET is still switching, generating
heat and switching losses.

When set, the output behaviour changes:
- Slider **1–99%** → linearly spans the dimmable range from `min_power` to `max_flat_threshold`
- Slider **100%** → jumps directly to `max_power` (fully on if 1.0, zero switching losses)

The HA UI always shows a clean 1–100% range regardless of where the threshold is set.

**Requires `gamma_correct: 0` (or `gamma_correct: 1`) on the light entity.**
The `acos` RMS compensation inside the component handles perceptual linearity for
phase-angle dimming. Gamma correction applied by the light entity transforms the state
value before it reaches the component, shifting threshold comparisons into the wrong space.
With `gamma_correct: 0`, the state seen by the component matches the HA slider position
exactly (0.0 = off, 1.0 = 100%).

---

### `half_cycle_offset`
Signed microsecond offset applied to alternate half-cycles to compensate for Vgs(th) mismatch
between the two back-to-back MOSFETs in a trailing-edge topology. Even identical part-number
devices can have Vgs(th) spread of ±0.5V within spec, causing slightly different turn-on
delays between positive and negative half-cycles. On capacitorless loads (e.g. LED strips with
a simple bridge rectifier and no bulk capacitance) this asymmetry is visible as flicker.

**How it works:**
- For `zc_method: edges` — half-cycle identity is determined by reading the ZC pin state at
  ISR time. This is fully deterministic and cannot drift. Pin LOW at interrupt = negative
  half-cycle (falling edge just fired on a high-on-positive NPN ZCD circuit).
- For `zc_method: pulse` or `inverted_pulse` — a toggle flag is used. Reliable because at
  60Hz with one interrupt per half-cycle there is no corruption source.

**Polarity convention (consistent across all dim methods):**
- Positive value → more conduction on the identified half-cycle
- Negative value → less conduction on the identified half-cycle

For `trailing`: offset is added to `disable_time_us` (larger = gate stays on longer = more conduction).
For `leading` / `leading_pulse`: offset is subtracted from `enable_time_us` (smaller = gate fires earlier = more conduction).

**Tuning procedure:**
1. Set dimmer to ~50% brightness
2. Connect oscilloscope across the load (DC side if load has a bridge rectifier)
3. Measure conduction window on each half-cycle
4. If positive half-cycle conduction is larger: use a negative `half_cycle_offset`
5. If negative half-cycle conduction is larger: use a positive `half_cycle_offset`
6. Step in increments of 10–20µs until both half-cycles show equal conduction

Range: -500 to +500µs. Default: 0 (disabled).

---

### `min_power` / `max_power`
Inherited from the `FloatOutput` base class. Clamp the output range. `max_power` is also
the jump target when `max_flat_threshold` is set and the slider is at 100%.

---

## Hardware notes

### Back-to-back MOSFET topology
Use `method: trailing`. With an independent gate bias supply (not derived from the load
side), full conduction at `max_power: 1.0` holds the gate high for the entire half-cycle
with no switching — zero switching losses. This is the primary advantage over single-MOSFET
bridge designs where full conduction is not achievable.

**Half-cycle asymmetry:** Even matched MOSFETs have unit-to-unit Vgs(th) variation. On
capacitorless loads this can cause visible flicker. Use `half_cycle_offset` to compensate.
A small film capacitor (100–470nF, 250V) across the DC bus after the load's bridge
rectifier also helps by damping switching transients.

### TRIAC topology
Use `method: leading_pulse` (default) or `method: leading`. The component is compatible
with standard TRIAC gate driver optocouplers (MOC3021 series — use non-zero-cross variants
for phase-angle control).

### ZCD circuit for sustained-level output (recommended)
An NPN transistor with base resistor and collector pull-up, or an H11A1 anti-parallel
dual-LED optocoupler, produces a sustained level suitable for `zc_method: edges`:
- High during positive half-cycle → LOW during negative half-cycle (NPN variant)
- Use `zc_method: edges` to detect both transitions

---

## The core change from upstream

```cpp
// upstream ac_dimmer — fixed to falling edge only:
this->zero_cross_pin_->attach_interrupt(..., gpio::INTERRUPT_FALLING_EDGE);

// advanced_ac_dimmer — selected at runtime from zc_method config:
switch (this->zc_method_) {
  case ZC_METHOD_PULSE:          intr_type = gpio::INTERRUPT_FALLING_EDGE; break;
  case ZC_METHOD_INVERTED_PULSE: intr_type = gpio::INTERRUPT_RISING_EDGE;  break;
  case ZC_METHOD_EDGES: default: intr_type = gpio::INTERRUPT_ANY_EDGE;     break;
}
```

All ISR logic (`timer_intr`, `gpio_intr`, `s_gpio_intr`) is otherwise based on upstream.

---

## License

Based on ESPHome's `ac_dimmer` component, licensed under the MIT License.
Modifications by Victor Vation.
