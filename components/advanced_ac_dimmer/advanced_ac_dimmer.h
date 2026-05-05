#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/output/float_output.h"

namespace esphome::advanced_ac_dimmer {

enum DimMethod { DIM_METHOD_LEADING_PULSE = 0, DIM_METHOD_LEADING, DIM_METHOD_TRAILING };

/// Zero crossing detection circuit type.
///   edges          - INTERRUPT_ANY_EDGE: both edges are zero crossings.
///                    Use with sustained-level circuits (e.g. H11A1-based: 0V positive
///                    half-cycle, 3.3V negative half-cycle). Default.
///   pulse          - INTERRUPT_FALLING_EDGE: active-low pulse at each zero crossing.
///                    Equivalent to the original upstream ac_dimmer behaviour.
///   inverted_pulse - INTERRUPT_RISING_EDGE: active-high pulse at each zero crossing.
enum ZcMethod { ZC_METHOD_EDGES = 0, ZC_METHOD_PULSE, ZC_METHOD_INVERTED_PULSE };

struct AcDimmerDataStore {
  /// Zero-cross pin
  ISRInternalGPIOPin zero_cross_pin;
  /// Zero-cross pin number - used to share ZC pin across multiple dimmers
  uint8_t zero_cross_pin_number;
  /// Output pin to write to
  ISRInternalGPIOPin gate_pin;
  /// Value of the dimmer - 0 to 65535.
  uint16_t value;
  /// Minimum power for activation
  uint16_t min_power;
  /// Time between the last two ZC pulses (half-cycle duration in µs)
  uint32_t cycle_time_us;
  /// Time (in micros()) of last ZC signal
  uint32_t crossed_zero_at;
  /// Time since last ZC pulse to enable gate pin. 0 means not set.
  uint32_t enable_time_us;
  /// Time since last ZC pulse to disable gate pin. 0 means no disable.
  uint32_t disable_time_us;
  /// Countdown of full half-cycles remaining to send on turn-on kickstart.
  /// 0 = no kickstart active. Decremented by gpio_intr() on each zero crossing.
  uint8_t init_cycle_count;
  /// Dimmer method
  DimMethod method;
  /// Zero crossing detection method — stored in ISR struct so gpio_intr() can
  /// select the correct half-cycle identification strategy for half_cycle_offset.
  ZcMethod zc_method;
  /// Signed offset in µs added to disable_time_us on alternate half-cycles to compensate
  /// Vgs(th) mismatch between back-to-back MOSFETs. Positive extends one half-cycle
  /// conduction; negative shortens it. Tune with oscilloscope. Trailing method only.
  /// Half-cycle identity detection depends on zc_method:
  ///   edges: pin state read at ISR time (deterministic, drift-free).
  ///   pulse/inverted_pulse: toggle flag (reliable since one interrupt per half-cycle
  ///   means no drift source in steady state).
  int16_t half_cycle_offset_us{0};
  /// Toggle flag for half-cycle tracking in pulse/inverted_pulse modes.
  bool half_cycle_toggle{false};

  uint32_t timer_intr(uint32_t now);

  void gpio_intr();
  static void s_gpio_intr(AcDimmerDataStore *store);
#ifdef USE_ESP32
  static void s_timer_intr();
#endif
};

class AcDimmer : public output::FloatOutput, public Component {
 public:
  void setup() override;

  void dump_config() override;
  void set_gate_pin(InternalGPIOPin *gate_pin) { gate_pin_ = gate_pin; }
  void set_zero_cross_pin(InternalGPIOPin *zero_cross_pin) { zero_cross_pin_ = zero_cross_pin; }
  /// Number of full half-cycles to send at full power when turning on from off.
  /// 0 disables the kickstart. 1 = equivalent to upstream init_with_half_cycle: true.
  void set_init_with_n_half_cycles(uint8_t n) { init_with_n_half_cycles_ = n; }
  void set_method(DimMethod method) { method_ = method; }
  /// Flat zone threshold (0.0 = disabled). When state >= threshold, output jumps
  /// directly to max_power_ (from the FloatOutput base class) instead of continuing
  /// to dim linearly. Eliminates the dead zone where the MOSFET is still switching
  /// but lamp output is perceptually indistinguishable from maximum.
  /// NOTE: requires gamma_correct: 0 (or 1) on the light entity — see write_state().
  void set_max_flat_threshold(float threshold) { max_flat_threshold_ = threshold; }
  /// Whether to apply acos RMS power compensation in write_state.
  /// Correct for resistive/incandescent loads where brightness ∝ RMS power.
  /// For LED lamps with constant-current switching drivers, brightness is proportional
  /// to conduction fraction (linear), so acos over-corrects. Set false for LEDs.
  /// Default: true (backward compatible with upstream ac_dimmer behaviour).
  void set_rms_correction(bool enabled) { rms_correction_ = enabled; }
  void set_zc_method(ZcMethod zc_method) { zc_method_ = zc_method; }
  /// Signed µs offset applied to alternate half-cycles to compensate MOSFET Vgs(th) mismatch.
  /// Range -500 to +500µs. Tune with oscilloscope. Only effective with method: trailing.
  void set_half_cycle_offset(int16_t offset_us) { half_cycle_offset_us_ = offset_us; }

 protected:
  void write_state(float state) override;

  InternalGPIOPin *gate_pin_;
  InternalGPIOPin *zero_cross_pin_;
  AcDimmerDataStore store_;
  uint8_t init_with_n_half_cycles_{0};
  float max_flat_threshold_{0.0f};  // 0.0 = disabled
  bool rms_correction_{true};
  ZcMethod zc_method_{ZC_METHOD_EDGES};
  int16_t half_cycle_offset_us_{0};
  DimMethod method_;
};

}  // namespace esphome::advanced_ac_dimmer
