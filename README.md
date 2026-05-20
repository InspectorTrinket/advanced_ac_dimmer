# Advanced AC Dimmer

An ESPHome external component for AC phase-angle dimming of mains-connected loads.
Designed for real-world installations where timing precision, load compatibility,
and flicker-free operation matter.

## Features

| Feature | Description |
|---|---|
| Flicker-free timer architecture | `esp_timer` one-shot pair per channel, hard-synced to zero-crossing |
| Zero crossing detection | `edges` / `pulse` / `inverted_pulse` — matches any ZCD circuit |
| Brightness curve | `rms` / `linear` / `logarithmic` — select per load type |
| Multi-half-cycle kickstart | Configurable count for LED drivers that need sustained power to ignite |
| Flat zone / dead zone elimination | `max_flat_threshold` — removes the region where the MOSFET switches but output is perceptually at maximum |
| Half-cycle offset (MOSFET mismatch compensation) | Corrects Vgs(th) asymmetry between back-to-back MOSFETs |
| Trailing method (back-to-back MOSFET) | Full half-cycle conduction at 100% — zero switching losses |
| Leading / leading pulse (TRIAC) | Standard phase-angle leading-edge control |
| Multi-channel shared ZC pin | Multiple dimmers on one ZCD circuit |

---

## How it works

### Phase-angle dimming

AC phase-angle dimming works by allowing the load to conduct only during a controlled
fraction of each mains half-cycle. The zero-crossing detector (ZCD) circuit signals the
start of each half-cycle. From that point, a timed delay determines when the gate is
switched — either:

- **Trailing edge** (back-to-back MOSFETs / full bridge rectifier + MOSFET): gate is 
  driven HIGH at the zero crossing and turned LOW after the conduction window elapses. 
  The load sees the leading portion of the half-cycle.
- **Leading edge** (TRIAC): gate is driven LOW at the zero crossing and pulsed HIGH after
  the delay has elapsed. The load sees the trailing portion of the half-cycle.

### Timer architecture

The component uses one pair of `esp_timer` one-shot timers per channel:

- **`enable_timer`** — fires after the leading delay (leading / leading_pulse methods only)
- **`disable_timer`** — fires after the conduction window to turn the gate off

On every zero-crossing, the GPIO ISR (`s_gpio_intr`) runs a strict **two-pass protocol**:

```
Zero-crossing ISR fires
│
├── Pass 1  (all channels sharing this ZC pin):
│     esp_timer_stop(enable_timer)
│     esp_timer_stop(disable_timer)
│     gate_pin = LOW              ← hard sync at the ZC instant, before anything else
│
└── Pass 2  (per channel):
      debounce check (< 3 ms since last ZC → discard spurious edge)
      measure half-cycle duration from esp_timer_get_time() delta
      │
      ├─ trailing:       gate = HIGH immediately
      │                  esp_timer_start_once(disable_timer, conduction_window_µs)
      │
      ├─ leading_pulse:  esp_timer_start_once(enable_timer, delay_µs)
      │                    └─ callback: gate = HIGH
      │                               esp_timer_start_once(disable_timer, 50 µs)
      │                                 └─ callback: gate = LOW
      │
      └─ leading:        esp_timer_start_once(enable_timer, delay_µs)
                           └─ callback: gate = HIGH
                              (gate stays HIGH until next ZC pass 1)
```

**Why this eliminates jitter** compared to a polling approach:

The previous generation of ESP32 AC dimmer components — including upstream `ac_dimmer` —
uses a GPTimer running at 1 MHz with a fixed 50 µs alarm tick. Every 50 µs it polls all
channels to check whether it is time to fire. This introduces ±50 µs of quantization error
on every gate transition. At low brightness where the conduction window is only a few
hundred microseconds wide, this is a significant fractional error that is visible as
flickering. Additionally, `micros()` (used for ZC timestamps) runs from a different hardware
counter than `esp_timer`, and the two can drift relative to each other, producing a
systematic phase error on top of the quantization noise.

This component eliminates both sources:

- `esp_timer_get_time()` is used for all ZC timestamps — the same counter that backs all
  `esp_timer` one-shots, so there is no clock-domain mismatch between timestamps and timer
  arming.
- Gate transitions fire at `esp_timer` callback resolution: ~1 µs with
  `ESP_TIMER_ISR` dispatch, ~10–50 µs with `ESP_TIMER_TASK` dispatch. Both are
  vastly better than the ±50 µs fixed-tick polling approach.
- Pass 1 of the two-pass ISR deasserts the gate at the zero-crossing instant
  unconditionally, before any new timer is armed. This closes the race condition where a
  stale `disable_timer` from the previous half-cycle could overlap with the freshly armed
  `enable_timer` of the new half-cycle.

### WiFi interference and timer dispatch

On single-core devices (ESP32-C3, ESP32-C6), the WiFi stack shares the same CPU with the
dimmer timers. With default WiFi modem-sleep enabled, the WiFi hardware wakes every DTIM
interval (typically 100–1000 ms) and runs a burst of driver work that can starve the
`esp_timer` service task, causing `disable_timer` to fire late and producing sporadic
bright pulses. Two mitigations:

```yaml
wifi:
  power_save_mode: none   # eliminates bursty WiFi wakeup events
```

```yaml
esp32:
  framework:
    type: esp-idf
    sdkconfig_options:
      CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD: "y"  # callbacks immune to task preemption
```

On dual-core devices (ESP32, ESP32-S3), the WiFi stack is pinned to core 0 by IDF defaults
and the dimmer ISR and timers run on core 1. WiFi task scheduling cannot preempt the dimmer
at all. Combined with `ESP_TIMER_ISR` dispatch, spurious bright pulses are effectively
eliminated.

---

## Installation

```yaml
external_components:
  - source: github://InspectorTrinket/advanced_ac_dimmer
    components: [advanced_ac_dimmer]
```

---

## Full configuration example

```yaml
output:
  - platform: advanced_ac_dimmer
    id: dimmer1
    gate_pin: GPIO05             # Required. GPIO connected to gate driver input.
    zero_cross_pin:              # Required. GPIO connected to ZCD circuit output.
      number: GPIO03
      allow_other_uses: true     # Required when sharing ZC pin between channels.
      mode:
        input: true
        pullup: false
      inverted: false
    method: trailing             # trailing | leading_pulse | leading
    zc_method: edges             # edges | pulse | inverted_pulse
    curve: rms                   # rms | linear | logarithmic
    init_with_n_half_cycles: 10  # 0–255, default 0
    kickstart_threshold: 0.10    # 0.01–0.99, optional
    min_power: 0.01              # 0.0–1.0, default 0.0
    max_power: 1.0               # 0.0–1.0, default 1.0
    max_flat_threshold: 0.63     # 0.01–0.99, optional
    half_cycle_offset: 0         # -500 to +500 µs, default 0

light:
  - platform: monochromatic
    name: "Living Room"
    output: dimmer1
    gamma_correct: 0             # Required when using max_flat_threshold. See below.
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
| `leading_pulse` | Brief gate pulse at dim point. TRIAC conducts until zero crossing | TRIAC (default) |
| `leading` | Gate held high from dim point until end of half-cycle | MOSFET mimicking TRIAC operation |
| `trailing` | Gate held high from start of half-cycle until dim point | Back-to-back MOSFET |

---

### `zc_method`
Selects the GPIO interrupt mode to match your ZCD circuit output.

| Value | Interrupt | Circuit type |
|---|---|---|
| `edges` | `ANY_EDGE` | Sustained-level circuit — 0V on positive half-cycle, Vcc on negative (60 Hz Square Wave). **Default.** |
| `pulse` | `FALLING_EDGE` | Active-low narrow pulse at each zero crossing (120 Hz low pulse train). |
| `inverted_pulse` | `RISING_EDGE` | Active-high narrow pulse at each zero crossing (120 Hz high pulse train). |

---

### `curve`
Controls how the normalised brightness value from the HA slider is mapped to a conduction
angle before being applied to the dimmer.

| Value | Transform | Best for |
|---|---|---|
| `rms` | acos(1 − 2s) / π | Resistive / incandescent loads. Corrects the nonlinear relationship between phase-angle conduction fraction and delivered RMS power. **Default.** |
| `linear` | s (identity) | Testing, or loads with their own linearisation (LEDs without driver, like mains LED strips). |
| `logarithmic` | log₁₀(1 + 9·s) | LED loads with constant-current switching drivers. Brightness is already linear with conduction fraction; the logarithmic curve allocates more steps to the low-brightness region where the eye is most sensitive. |

All three curves map the input range [0, 1] to the output range [0, 1].

---

### `init_with_n_half_cycles`
Number of full half-cycles at maximum conduction sent when the output turns on from off.
Provides sustained power to help LED drivers whose internal capacitors need time to charge
before the LED will conduct. `0` disables the feature (default).

Increase the value until the lamp ignites reliably on every turn-on.

---

### `kickstart_threshold`
Suppresses kickstart when the target brightness is at or above this value. Above the
threshold the lamp self-starts reliably from the phase-angle conduction alone — no
full-power flash is needed.

Expressed as a normalised slider position (0.01–0.99), compared against the raw state
before curve compensation. `kickstart_threshold: 0.17` means the HA slider at 17% or above.

**Transition-aware:** if a transition ramps the brightness upward through the threshold
while kickstart is still counting down, kickstart is cancelled at the crossing point. The
lamp is already conducting at that brightness level so the cancellation is seamless.

**Tuning procedure:**
1. With `init_with_n_half_cycles: 0` (kickstart disabled), find the lowest slider position
   at which the lamp turns on reliably from off
2. Set `kickstart_threshold` to that value
3. Re-enable `init_with_n_half_cycles` — kickstart will now only fire below the threshold

---

### `max_flat_threshold`
Eliminates the flat zone — the region near maximum brightness where the lamp output is
perceptually indistinguishable from full on, but the MOSFET is still switching, generating 
heat and switching losses.

When set, the slider behaviour changes:
- Slider **1–99%** → linearly spans the dimmable range from `min_power` to `max_flat_threshold`
- Slider **100%** → jumps directly to `max_power` (fully on if 1.0, zero switching losses)

The HA UI shows a clean 1–100% range regardless of where the threshold is set.

**Requires `gamma_correct: 0` on the light entity.** The curve compensation inside the
component handles perceptual linearity. Gamma correction applied by the light entity
before the value reaches the component shifts the threshold comparisons into the wrong
space — the threshold would trigger at the wrong slider position.

---

### `half_cycle_offset`
Signed microsecond offset applied to alternate half-cycles to compensate for Vgs(th)
mismatch between the two back-to-back MOSFETs in a trailing-edge topology. Even identical
part-number devices can have Vgs(th) spread of ±0.5 V within spec, causing slightly
different turn-on delays between positive and negative half-cycles.

**Half-cycle identity detection:**
- `zc_method: edges` — ZC pin state is read at ISR time. Fully deterministic and
  drift-free. Pin LOW at interrupt = negative half-cycle.
- `zc_method: pulse` or `inverted_pulse` — a toggle flag is used. Reliable because at
  60 Hz with one interrupt per half-cycle there is no corruption source.

**Polarity convention:**
- Positive value → more conduction on the identified half-cycle
- Negative value → less conduction on the identified half-cycle

**Tuning procedure:**
1. Set dimmer to ~50% brightness
2. Connect oscilloscope across the load
3. Measure conduction window on each half-cycle
4. If positive half-cycle conduction is larger: use a negative `half_cycle_offset`
5. If negative half-cycle conduction is larger: use a positive `half_cycle_offset`
6. Step in increments of 10–20 µs until both half-cycles match

Range: −500 to +500 µs. Default: 0 (disabled).

---

### `min_power` / `max_power`
Inherited from the `FloatOutput` base class. Clamp the effective output range.
`max_power` is also the jump target when `max_flat_threshold` is set and the slider is
at 100%.

---

## Diagnostics

### Mains frequency sensor

The component measures the mains half-cycle duration on every zero-crossing and exposes
it via `get_frequency_hz()`. Use this to diagnose ZCD circuit issues — frequency glitches,
loss of sync, or spurious edges show up immediately as deviations from 60 Hz (or 50 Hz).

Add to your YAML `sensor:` section:

```yaml
sensor:
  - platform: template
    name: "Mains Frequency"
    device_class: frequency
    state_class: measurement
    unit_of_measurement: Hz
    accuracy_decimals: 2
    entity_category: diagnostic
    update_interval: 10s
    lambda: return id(dimmer1).get_frequency_hz();
```

Returns `0.0` until the first zero-crossing is detected. When a glitch occurs, watch for
the frequency jumping away from the nominal value, dropping to 0, or oscillating — this
identifies whether the ZCD circuit is losing signal, generating spurious edges, or simply
producing noise that the 3 ms debounce is catching intermittently.

---

## Hardware notes

### Back-to-back MOSFET topology

Use `method: trailing`. At `max_power: 1.0` the gate is held HIGH for the entire half-cycle
with no switching — zero switching losses. This is the primary advantage over TRIAC or
single-MOSFET designs.

Mismatched Vgs(th) between the two MOSFETs produces asymmetric conduction on positive and
negative half-cycles. Use `half_cycle_offset` to compensate.

### TRIAC topology

Use `method: leading_pulse` (default). Use non-zero-cross TRIAC gate driver optocouplers 
(e.g. MOC3021 series) — zero-cross variants suppress firing at the phase angle set by the 
dimmer.

---

## License

MIT License.  
Modifications and timer architecture by InspectorTrinket.
