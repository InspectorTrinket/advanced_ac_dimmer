#pragma once

/**
 * advanced_ac_dimmer — ESP32 AC phase-angle dimmer for ESPHome
 *
 * Timer architecture (revised, flicker-free):
 *   Each channel owns two esp_timer one-shots:
 *     enable_timer  — ZC → gate HIGH  (leading / leading_pulse methods)
 *     disable_timer — gate HIGH → LOW (trailing: armed from ZC ISR;
 *                                      leading_pulse: armed from enable_timer cb)
 *
 *   On every zero-crossing s_gpio_intr() runs a strict two-pass protocol:
 *     Pass 1: stop both timers + drive gate LOW for EVERY channel sharing the ZC pin.
 *     Pass 2: call gpio_intr() per channel to compute timing and arm timers.
 *
 *   This guarantees the gate is deasserted at the zero crossing instant before
 *   any new timer is armed, eliminating a race that causes jitter.
 *
 *   Timers use ESP_TIMER_ISR dispatch when CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD
 *   is enabled (~1µs jitter); fall back to ESP_TIMER_TASK otherwise (~10–50µs).
 *
 *   esp_timer_get_time() is used for all ZC timestamps — consistent timebase
 *   with esp_timer internals, eliminating the drift that occurred when micros()
 *   and GPTimer ran from different clock sources.
 *
 * Other features:
 *   trailing / leading_pulse / leading methods
 *   edges / pulse / inverted_pulse ZC detection
 *   N-half-cycle kickstart (init_with_n_half_cycles)
 *   Brightness curve: rms / linear / logarithmic (curve option)
 *   Flat zone elimination (max_flat_threshold)
 *   Half-cycle offset for MOSFET Vgs(th) mismatch compensation
 *   Multi-channel shared ZC pin
 */

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/output/float_output.h"

#ifdef USE_ESP32
#include "esp_timer.h"
#endif

namespace esphome::advanced_ac_dimmer {

enum DimMethod { DIM_METHOD_LEADING_PULSE = 0, DIM_METHOD_LEADING, DIM_METHOD_TRAILING };

/// Zero crossing detection circuit type.
///   edges          - INTERRUPT_ANY_EDGE: both edges are zero crossings.
///                    Use with sustained-level circuit — 0V on positive half-cycle, Vcc on negative (60 Hz Square Wave).
///   pulse          - INTERRUPT_FALLING_EDGE: active-low narrow pulse at each zero crossing (120 Hz low pulse train)
///   inverted_pulse - INTERRUPT_RISING_EDGE: active-high narrow pulse at each zero crossing (120 Hz high pulse train).
enum ZcMethod { ZC_METHOD_EDGES = 0, ZC_METHOD_PULSE, ZC_METHOD_INVERTED_PULSE };

/// Brightness-to-conduction-angle mapping curve.
///   linear      - No compensation. Conduction fraction maps linearly to value.
///                 Use when the load or driver already provides its own
///                 linearisation, or for testing.
///   rms         - acos(1 − 2s)/π compensation. Corrects the nonlinear
///                 relationship between phase-angle conduction fraction and RMS
///                 power. Correct for resistive / incandescent loads where
///                 brightness ~ RMS power. Default.
///   logarithmic - log₁₀(1 + 9·s) perceptual curve. Allocates more dimmer
///                 steps at low brightness where the eye is most sensitive.
///                 Best for LED loads with constant-current switching drivers,
///                 where brightness is already linear with conduction fraction
///                 so the rms acos transform over-corrects.
enum DimCurve { DIM_CURVE_RMS = 0, DIM_CURVE_LINEAR, DIM_CURVE_LOGARITHMIC };

struct AcDimmerDataStore {
  // ── Input / output handles ────────────────────────────────────────────────
  ISRInternalGPIOPin zero_cross_pin;
  uint8_t zero_cross_pin_number;
  ISRInternalGPIOPin gate_pin;

  // ── Fields shared between main-context writes and ISR reads ──────────────
  // volatile prevents the compiler from caching them across context boundaries.
  volatile uint16_t value;           ///< Brightness: 0=off, 65535=fully on
  volatile uint16_t min_power;       ///< Min conduction, stored as 0–1000 (per-mille)
  volatile uint32_t cycle_time_us;   ///< Last measured half-cycle duration [µs]
  volatile int64_t  last_zc_time;    ///< esp_timer_get_time() at last valid ZC [µs]
  volatile uint8_t  init_cycle_count;///< Kickstart half-cycles remaining; 0 = inactive
  /// Pre-computed curve-transformed kickstart threshold for ISR comparison.
  /// 0 = threshold disabled. Written once by write_state(); read by gpio_intr()
  /// to cancel kickstart as soon as value rises above it during a transition.
  volatile uint16_t kickstart_threshold_value{0};

  // ── Per-channel esp_timer one-shots (created once in setup()) ────────────
#ifdef USE_ESP32
  esp_timer_handle_t enable_timer;   ///< ZC → gate HIGH  (leading / leading_pulse)
  esp_timer_handle_t disable_timer;  ///< gate HIGH → LOW
  /// Pulse width for leading_pulse: written in gpio_intr(), read in enable_timer_cb().
  volatile uint32_t pending_disable_us;
#endif

  // ── Configuration (written once from setup(), read by ISR) ───────────────
  DimMethod method;
  ZcMethod  zc_method;
  int16_t   half_cycle_offset_us{0};
  bool      half_cycle_toggle{false};

  // ── ISR methods ──────────────────────────────────────────────────────────
  /// Compute timing for this half-cycle and arm the appropriate timer(s).
  /// Called from s_gpio_intr() *after* pass 1 has already stopped timers and
  /// driven the gate LOW.
  void gpio_intr();

  /// GPIO ISR entry point. Implements the two-pass protocol and dispatches
  /// gpio_intr() for every channel that shares this ZC pin.
  static void s_gpio_intr(AcDimmerDataStore *store);

#ifdef USE_ESP32
  /// esp_timer callback: fires gate HIGH (and for leading_pulse, arms disable_timer).
  static void enable_timer_cb(void *arg);
  /// esp_timer callback: drives gate LOW.
  static void disable_timer_cb(void *arg);
#endif
};

class AcDimmer : public output::FloatOutput, public Component {
 public:
  void setup() override;
  void dump_config() override;

  void set_gate_pin(InternalGPIOPin *gate_pin) { gate_pin_ = gate_pin; }
  void set_zero_cross_pin(InternalGPIOPin *zero_cross_pin) { zero_cross_pin_ = zero_cross_pin; }

  /// Number of full half-cycles at maximum conduction sent when the output turns on
  /// from off. 0 disables kickstart. 1 is equivalent to upstream init_with_half_cycle.
  void set_init_with_n_half_cycles(uint8_t n) { init_with_n_half_cycles_ = n; }

  /// Kickstart threshold (0.0 = disabled). Kickstart is suppressed when the target
  /// brightness is at or above this value — the lamp self-starts reliably at those
  /// levels without a full-power flash. Expressed as a normalised slider position
  /// (0.0–1.0), compared against the raw state before curve compensation.
  ///
  /// Transition-aware: if a transition brings the brightness through the threshold
  /// during kickstart, kickstart is cancelled at the moment the threshold is crossed.
  /// The lamp is already conducting at that point so the cancellation is seamless.
  void set_kickstart_threshold(float threshold) { kickstart_threshold_ = threshold; }

  void set_method(DimMethod method) { method_ = method; }

  /// Flat zone threshold (0.0 = disabled). Slider 100% jumps to max_power_;
  /// slider 1–99% spans min_power to max_flat_threshold linearly.
  /// Requires gamma_correct: 0 on the light entity.
  void set_max_flat_threshold(float threshold) { max_flat_threshold_ = threshold; }

  /// Brightness curve applied in write_state().
  ///   rms         — acos RMS compensation (default, resistive/incandescent loads)
  ///   linear      — no compensation
  ///   logarithmic — log₁₀(1+9s) perceptual curve (LED loads)
  void set_curve(DimCurve curve) { curve_ = curve; }

  void set_zc_method(ZcMethod zc_method) { zc_method_ = zc_method; }

  /// Signed µs offset applied to alternate half-cycles to compensate MOSFET Vgs(th)
  /// mismatch. Range ±500µs. Tune with oscilloscope.
  void set_half_cycle_offset(int16_t offset_us) { half_cycle_offset_us_ = offset_us; }

 protected:
  void write_state(float state) override;

  InternalGPIOPin *gate_pin_;
  InternalGPIOPin *zero_cross_pin_;
  AcDimmerDataStore store_;
  uint8_t init_with_n_half_cycles_{0};
  float   kickstart_threshold_{0.0f};  ///< 0.0 = disabled (kickstart always fires)
  float   max_flat_threshold_{0.0f};
  DimCurve curve_{DIM_CURVE_RMS};
  ZcMethod zc_method_{ZC_METHOD_EDGES};
  int16_t  half_cycle_offset_us_{0};
  DimMethod method_;
};

}  // namespace esphome::advanced_ac_dimmer
