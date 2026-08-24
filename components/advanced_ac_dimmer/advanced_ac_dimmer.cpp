#include "advanced_ac_dimmer.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <cmath>
#include <numbers>

#ifdef USE_ESP32
#include "esp_timer.h"
#include "esp_attr.h"
#endif

namespace esphome::advanced_ac_dimmer {

static const char *const TAG = "advanced_ac_dimmer";

// ── ISR-accessible global ─────────────────────────────────────────────────────
// DRAM_ATTR: the array is read by s_gpio_intr() (GPIO ISR context).
// Without it, a flash-cache miss during NVS/OTA activity stalls the ISR and
// produces timing spikes that are visible as flicker.
static DRAM_ATTR AcDimmerDataStore *all_dimmers[32];  // NOLINT

// ── Constants ─────────────────────────────────────────────────────────────────

/// Minimum time in µs between two accepted zero-crossing edges.
/// Rejects MOSFET switching noise and optocoupler bounce that would otherwise
/// reset the timer chain mid-half-cycle and cause missed gate pulses → flicker.
/// At 60 Hz a half-cycle is 8333 µs; 3000 µs is well below that but above any
/// legitimate glitch from the AC waveform.
static constexpr uint32_t ZC_DEBOUNCE_US = 3000;

/// Minimum time in µs the gate is held high for a leading_pulse or to ensure a
/// trailing gate-on pulse is wide enough for the MOSFET to fully turn on.
static constexpr uint32_t GATE_ENABLE_TIME = 50;

// ── ESP_TIMER dispatch selection ──────────────────────────────────────────────
// ESP_TIMER_ISR dispatch gives ~1 µs callback latency but requires the IDF
// Kconfig option CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD=y.
// Arduino Core 3.x on ESP32-C3 does NOT enable this option by default;
// Fall back to ESP_TIMER_TASK dispatch (~10–50 µs).
#ifdef USE_ESP32
#ifdef CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD
  #define DIMMER_TIMER_DISPATCH  ESP_TIMER_ISR
#else
  #define DIMMER_TIMER_DISPATCH  ESP_TIMER_TASK
#endif
#endif

// ── Timer callbacks ───────────────────────────────────────────────────────────

#ifdef USE_ESP32

/// enable_timer callback — fires gate HIGH after the leading delay has elapsed.
/// For leading_pulse: chains into disable_timer to produce a fixed-width gate pulse.
/// For leading: gate stays high until the next zero-crossing clears it in pass 1.
///
/// IRAM_ATTR: must reside in IRAM so it can run during flash operations without
/// a cache miss introducing unpredictable latency.
void IRAM_ATTR AcDimmerDataStore::enable_timer_cb(void *arg) {
  auto *store = reinterpret_cast<AcDimmerDataStore *>(arg);
  store->gate_pin.digital_write(true);
  if (store->method == DIM_METHOD_LEADING_PULSE) {
    esp_timer_start_once(store->disable_timer, store->pending_disable_us);
  }
  // DIM_METHOD_LEADING: gate held high until next ZC s_gpio_intr pass 1 clears it.
}

/// disable_timer callback — drives gate LOW.
void IRAM_ATTR AcDimmerDataStore::disable_timer_cb(void *arg) {
  auto *store = reinterpret_cast<AcDimmerDataStore *>(arg);
  store->gate_pin.digital_write(false);
}

#endif  // USE_ESP32

// ── Zero-crossing handler ─────────────────────────────────────────────────────

/// gpio_intr: called by s_gpio_intr (pass 2) after timers have been stopped and
/// gate driven LOW. Computes timing for this half-cycle and arms the appropriate
/// esp_timer one-shot(s).
///
/// All timing is derived from esp_timer_get_time() which is the same high-resolution
/// timebase that esp_timer uses internally, eliminating clock-domain mismatch.
void IRAM_ATTR HOT AcDimmerDataStore::gpio_intr() {
#ifndef USE_ESP32
  // Non-ESP32 targets (ESP8266): not supported with this timer architecture.
  return;
#else
  // ── Timestamp & debounce ─────────────────────────────────────────────────
  // esp_timer_get_time() returns µs since boot from the same hardware counter
  // that backs all esp_timer one-shots — no drift between ZC timestamps and
  // timer arming.
  int64_t now = esp_timer_get_time();

  if (this->last_zc_time != 0) {
    int64_t elapsed = now - this->last_zc_time;

    // Debounce: reject edges within ZC_DEBOUNCE_US of the previous accepted edge.
    // Eliminates MOSFET switching transients and optocoupler bounce that would
    // otherwise start a new timer chain mid-half-cycle.
    if (elapsed < static_cast<int64_t>(ZC_DEBOUNCE_US)) {
      return;
    }

    // Update half-cycle duration from the measured interval.
    // Valid range 5 ms – 15 ms covers 33 Hz – 100 Hz with margin.
    if (elapsed > 5000 && elapsed < 15000) {
      this->cycle_time_us = static_cast<uint32_t>(elapsed);
    }
  }
  this->last_zc_time = now;

  // ── Arm output for this half-cycle ───────────────────────────────────────

  // Fully on: gate HIGH immediately, no timer needed.
  if (this->value == 65535) {
    this->gate_pin.digital_write(true);
    return;
  }

  // Kickstart: drive gate high for the full half-cycle to charge LED driver caps.
  if (this->init_cycle_count > 0) {
    // Cancel kickstart if value has risen above the threshold during a transition.
    // write_state() updates store_.value at the ESPHome loop rate (~60 Hz); gpio_intr()
    // checks here at 120 Hz, so cancellation happens within one half-cycle (8.33 ms)
    // of write_state() committing a value above the threshold — far faster than relying
    // on write_state() alone, which would miss the window for long transitions.
    if (this->kickstart_threshold_value > 0 && this->value >= this->kickstart_threshold_value) {
      this->init_cycle_count = 0;
      // Fall through to normal phase-angle dimming below.
    } else {
      this->init_cycle_count--;
      this->gate_pin.digital_write(true);
      // Arm disable_timer to pull gate LOW at end of half-cycle.
      // If cycle_time_us is not yet known (very first ZC), skip — the next ZC's
      // pass 1 will clear the gate.
      if (this->cycle_time_us > 0) {
        esp_timer_start_once(this->disable_timer, this->cycle_time_us);
      }
      return;
    }
  }

  // Fully off or no timing data yet: gate stays LOW (already done in pass 1).
  if (this->value == 0 || this->cycle_time_us == 0) {
    return;
  }

  // ── Timing computation ───────────────────────────────────────────────────
  // min_power is stored as per-mille (0–1000); convert to µs offset.
  uint32_t min_us = this->cycle_time_us * this->min_power / 1000;

  // Half-cycle offset — compensates Vgs(th) mismatch between back-to-back MOSFETs.
  bool apply_offset = false;
  if (this->half_cycle_offset_us != 0) {
    if (this->zc_method == ZC_METHOD_EDGES) {
      // Read ZC pin state at ISR time: fully deterministic, cannot drift.
      // Pin LOW = falling edge just fired = start of negative half-cycle
      // (for a high-on-positive H11A1-based ZCD circuit).
      apply_offset = !this->zero_cross_pin.digital_read();
    } else {
      // pulse / inverted_pulse: one interrupt per half-cycle, toggle is reliable.
      this->half_cycle_toggle = !this->half_cycle_toggle;
      apply_offset = this->half_cycle_toggle;
    }
  }

  if (this->method == DIM_METHOD_TRAILING) {
    // ── Trailing edge (back-to-back MOSFET) ─────────────────────────────
    // Gate turns on immediately at ZC; disable_timer turns it off after the
    // computed conduction window. Positive half_cycle_offset extends conduction
    // (larger disable time) on the identified half-cycle.
    uint32_t base_disable = this->value * (this->cycle_time_us - min_us) / 65535 + min_us;
    int32_t  adj          = static_cast<int32_t>(base_disable);
    if (apply_offset) {
      adj += static_cast<int32_t>(this->half_cycle_offset_us);
    }
    uint32_t disable_us = static_cast<uint32_t>(
        std::max(static_cast<int32_t>(GATE_ENABLE_TIME + 1), adj));

    this->gate_pin.digital_write(true);
    esp_timer_start_once(this->disable_timer, disable_us);

  } else {
    // ── Leading edge (TRIAC or leading-edge MOSFET) ───────────────────
    // Gate is LOW at ZC (done in pass 1). enable_timer fires after the leading
    // delay; its callback drives gate HIGH and — for leading_pulse — chains
    // disable_timer for a fixed-width gate pulse.
    // Positive half_cycle_offset means more conduction = smaller enable delay.
    uint32_t base_enable = std::max(static_cast<uint32_t>(1),
        ((65535 - this->value) * (this->cycle_time_us - min_us)) / 65535);
    int32_t  adj         = static_cast<int32_t>(base_enable);
    if (apply_offset) {
      adj -= static_cast<int32_t>(this->half_cycle_offset_us);
    }
    uint32_t enable_us = static_cast<uint32_t>(std::max(static_cast<int32_t>(1), adj));

    if (this->method == DIM_METHOD_LEADING_PULSE) {
      // enable_timer_cb will use this to start disable_timer.
      this->pending_disable_us = GATE_ENABLE_TIME;
    }
    // DIM_METHOD_LEADING: gate stays high after enable_timer_cb fires; the next
    // ZC's pass 1 stops enable_timer (if still pending) and drives gate LOW.

    esp_timer_start_once(this->enable_timer, enable_us);
  }
#endif  // USE_ESP32
}

/// GPIO ISR entry point — two-pass protocol.
///
/// Pass 1 (loop): for every channel sharing this ZC pin, stop both timers
///   and drive the gate LOW immediately. This is the hard synchronisation
///   point — all outputs are deasserted at the same instant the zero-crossing
///   is detected, before any new timer is armed.
///
/// Pass 2 (loop): call gpio_intr() on each channel to compute timing and
///   arm the appropriate esp_timer one-shot(s) for the new half-cycle.
///
/// The two-pass split eliminates a race condition where a stale disable_timer 
/// from the previous half-cycle could overlap with the freshly armed enable_timer 
/// of the new half-cycle.
void IRAM_ATTR HOT AcDimmerDataStore::s_gpio_intr(AcDimmerDataStore *store) {
#ifdef USE_ESP32
  // ── Pass 1: stop timers + gate LOW ─────────────────────────────────────
  for (auto *dimmer : all_dimmers) {
    if (dimmer == nullptr) break;
    if (dimmer->zero_cross_pin_number == store->zero_cross_pin_number) {
      esp_timer_stop(dimmer->enable_timer);
      esp_timer_stop(dimmer->disable_timer);
      dimmer->gate_pin.digital_write(false);
    }
  }
  // ── Pass 2: arm timers for the new half-cycle ───────────────────────────
  for (auto *dimmer : all_dimmers) {
    if (dimmer == nullptr) break;
    if (dimmer->zero_cross_pin_number == store->zero_cross_pin_number) {
      dimmer->gpio_intr();
    }
  }
#else
  // ESP8266: single-pass fallback (no esp_timer support in this implementation)
  for (auto *dimmer : all_dimmers) {
    if (dimmer == nullptr) break;
    if (dimmer->zero_cross_pin_number == store->zero_cross_pin_number) {
      dimmer->gpio_intr();
    }
  }
#endif
}

// ── AcDimmer::setup() ─────────────────────────────────────────────────────────

void AcDimmer::setup() {
  // Determine whether this instance is the first to register this ZC pin.
  // Only the first registration calls zero_cross_pin_->setup() and attaches
  // the ISR — re-doing it on a second dimmer would reset GPIO config and
  // detach the interrupt already installed by the first, killing all output.
  bool setup_zero_cross_pin = true;

  // Find the free slot but do not publish into it yet — see the publish
  // step at the end of this function for why.
  int free_slot = -1;
  for (int i = 0; i < static_cast<int>(sizeof(all_dimmers) / sizeof(all_dimmers[0])); i++) {
    if (all_dimmers[i] == nullptr) {
      free_slot = i;
      break;
    }
    if (all_dimmers[i]->zero_cross_pin_number == this->zero_cross_pin_->get_pin()) {
      setup_zero_cross_pin = false;
    }
  }

  // ── Gate pin ─────────────────────────────────────────────────────────────
  this->gate_pin_->setup();
  this->store_.gate_pin            = this->gate_pin_->to_isr();
  this->store_.zero_cross_pin_number = this->zero_cross_pin_->get_pin();

  // ── Store configuration ──────────────────────────────────────────────────
  this->store_.min_power           = static_cast<uint16_t>(this->min_power_ * 1000);
  this->min_power_                 = 0;
  this->store_.method              = this->method_;
  this->store_.zc_method           = this->zc_method_;
  this->store_.half_cycle_offset_us = this->half_cycle_offset_us_;

  // ── Initialise timer-related fields ─────────────────────────────────────
  this->store_.value               = 0;
  this->store_.cycle_time_us       = 0;
  this->store_.last_zc_time        = 0;
  this->store_.init_cycle_count    = 0;
  this->store_.was_explicitly_off  = true;  // first turn-on after boot arms kickstart

  // Pre-compute kickstart threshold value in post-curve space so the ISR can
  // compare against store_.value without floating-point. Computed once here —
  // it is constant and must not be written from write_state() to avoid
  // cache-line contention between the main task and the ISR.
  if (this->kickstart_threshold_ > 0.0f) {
    float min_p   = this->min_power_ == 0.0f
                        ? this->store_.min_power / 1000.0f
                        : this->min_power_;
    float ceiling = (this->max_flat_threshold_ > 0.0f)
                        ? this->max_flat_threshold_
                        : this->max_power_;
    float remapped = min_p + this->kickstart_threshold_ * (ceiling - min_p);
    // Apply the same curve transform used in write_state().
    if (this->curve_ == DIM_CURVE_RMS) {
      remapped = std::acos(1.0f - (2.0f * remapped)) / static_cast<float>(std::numbers::pi);
    } else if (this->curve_ == DIM_CURVE_LOGARITHMIC) {
      remapped = std::log10(1.0f + 9.0f * remapped);
    }
    this->store_.kickstart_threshold_value =
        static_cast<uint16_t>(roundf(remapped * 65535.0f));
  } else {
    this->store_.kickstart_threshold_value = 0;
  }
#ifdef USE_ESP32
  this->store_.enable_timer        = nullptr;
  this->store_.disable_timer       = nullptr;
  this->store_.pending_disable_us  = GATE_ENABLE_TIME;
#endif

  // ── ZC pin interrupt (first dimmer on this pin only) ─────────────────────
  if (setup_zero_cross_pin) {
    this->zero_cross_pin_->setup();
    gpio::InterruptType intr_type;
    switch (this->zc_method_) {
      case ZC_METHOD_PULSE:
        intr_type = gpio::INTERRUPT_FALLING_EDGE;
        break;
      case ZC_METHOD_INVERTED_PULSE:
        intr_type = gpio::INTERRUPT_RISING_EDGE;
        break;
      case ZC_METHOD_EDGES:
      default:
        intr_type = gpio::INTERRUPT_ANY_EDGE;
        break;
    }
    this->zero_cross_pin_->attach_interrupt(&AcDimmerDataStore::s_gpio_intr,
                                            &this->store_, intr_type);
  }
  // Always initialise zero_cross_pin ISR handle — gpio_intr() calls digital_read()
  // on it for half-cycle polarity detection even on shared-pin dimmers.
  this->store_.zero_cross_pin = this->zero_cross_pin_->to_isr();

#ifdef USE_ESP32
  // ── Create per-channel esp_timer one-shots ───────────────────────────────
  // Two timers per channel — no shared global timer, no polling.
  // dispatch_method: ESP_TIMER_ISR gives ~1 µs callback latency when the IDF
  // Kconfig option CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD is enabled
  // (requires manually adding it to sdkconfig.esphome or using IDF >= 5.1 with
  // Arduino Core >= 3.x built with that option). Falls back to ESP_TIMER_TASK
  {
    esp_timer_create_args_t args = {};
    args.dispatch_method         = DIMMER_TIMER_DISPATCH;
    args.skip_unhandled_events   = false;

    args.callback                = &AcDimmerDataStore::enable_timer_cb;
    args.arg                     = &this->store_;
    args.name                    = "dimmer_en";
    esp_err_t err = esp_timer_create(&args, &this->store_.enable_timer);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to create enable_timer (err %d)", err);
      this->mark_failed();
      return;
    }

    args.callback                = &AcDimmerDataStore::disable_timer_cb;
    args.name                    = "dimmer_dis";
    err = esp_timer_create(&args, &this->store_.disable_timer);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to create disable_timer (err %d)", err);
      esp_timer_delete(this->store_.enable_timer);
      this->store_.enable_timer = nullptr;
      this->mark_failed();
      return;
    }
  }
#endif

  // Publish: make this store visible to the ISR only now that gate_pin,
  // zero_cross_pin, every store_ field, and both esp_timer handles are valid.
  // If the ZC pin is shared and already owned by a previously set-up dimmer,
  // real zero-crossings are firing continuously from the moment that first
  // dimmer's setup() attached the interrupt — publishing earlier (as the
  // original single-pass registration did) would let the ISR call
  // gpio_intr() / esp_timer_stop() / gate writes against fields on this
  // store that are not yet initialised. This also means a dimmer whose
  // esp_timer_create() calls failed above is never published, so a failed
  // channel does not leave a stale nullptr-timer entry for pass 1 to call
  // esp_timer_stop() on forever afterward.
  if (free_slot >= 0) {
    all_dimmers[free_slot] = &this->store_;
  }
}

// ── write_state() ─────────────────────────────────────────────────────────────

void AcDimmer::write_state(float state) {
  uint16_t new_value;

  // apply_curve: maps a normalised brightness value [0,1] through the selected
  // curve, then converts to the 0–65535 internal scale.
  //
  // DIM_CURVE_RMS:
  //   acos(1 − 2s)/π — corrects the nonlinear relationship between phase-angle
  //   conduction fraction and delivered RMS power. Appropriate for resistive /
  //   incandescent loads where perceived brightness ~ RMS power.
  //
  // DIM_CURVE_LINEAR:
  //   Identity. Conduction fraction equals the normalised slider value directly.
  //   Use when the load already linearises (or for diagnostic / testing purposes).
  //
  // DIM_CURVE_LOGARITHMIC:
  //   log₁₀(1 + 9·s) — perceptual curve that allocates more dimmer steps to the
  //   low-brightness region where the eye is most sensitive. Best for LED loads
  //   with constant-current switching drivers, whose brightness is already linear
  //   with conduction fraction (making the acos RMS curve over-compensate).
  //   Formula from IES perceptual linearisation literature.
  auto apply_curve = [&](float s) -> uint16_t {
    switch (this->curve_) {
      case DIM_CURVE_RMS:
        s = std::acos(1.0f - (2.0f * s)) / static_cast<float>(std::numbers::pi);
        break;
      case DIM_CURVE_LOGARITHMIC:
        s = std::log10(1.0f + 9.0f * s);  // maps [0,1] → [0,1], log-distributed
        break;
      case DIM_CURVE_LINEAR:
      default:
        break;  // s unchanged
    }
    return static_cast<uint16_t>(roundf(s * 65535.0f));
  };

  if (state == 0.0f) {
    new_value = 0;

  } else if (this->max_flat_threshold_ > 0.0f) {
    // Flat zone mode: slider 1–99% spans [min_power, max_flat_threshold];
    // slider 100% (state == max_power_) jumps directly to max_power_.
    if (state >= this->max_power_ - 1e-5f) {
      new_value = (this->max_power_ >= 1.0f) ? 65535 : apply_curve(this->max_power_);
    } else {
      float remapped = (state / this->max_power_) * this->max_flat_threshold_;
      new_value      = apply_curve(remapped);
    }
  } else {
    new_value = apply_curve(state);
  }

  // ── Kickstart threshold logic ────────────────────────────────────────────
  // Threshold is evaluated against the raw state (pre-curve slider position)
  // so the configured value matches the HA slider percentage directly.
  // kickstart_threshold_value is pre-computed once in setup() — writing it
  // here on every call caused cache-line contention with the ISR reading it
  // at 120 Hz, producing a perceptible kink when dimming through the threshold.
  //
  // ARM (turning on from off): suppress kickstart immediately when the target
  // is already above threshold — handles instant turn-on with no transition.
  // For transitions the first write_state call carries a tiny interpolated
  // value below the threshold, so kickstart arms; gpio_intr() cancels it
  // within 8.33 ms of store_.value rising above kickstart_threshold_value.
  bool above_threshold = (this->kickstart_threshold_ > 0.0f &&
                          state >= this->kickstart_threshold_);

  if (new_value == 0) {
    // Mark as explicitly off only when the output is commanded to zero.
    // This distinguishes a real off command from a momentary zero caused by
    // HA state restoration, WiFi reconnection glitches, or script races —
    // none of which should re-arm kickstart on the next non-zero write.
    this->store_.was_explicitly_off = true;
  } else {
    if (this->store_.was_explicitly_off && !above_threshold) {
      // Arm kickstart only on the first non-zero write after a confirmed off,
      // and only if the target brightness is below the kickstart threshold.
      ESP_LOGD(TAG, "Kickstart armed (%u half-cycles)", this->init_with_n_half_cycles_);
      this->store_.init_cycle_count = this->init_with_n_half_cycles_;
    }
    // Clear the flag on any non-zero write — including above-threshold turn-ons.
    // Without this, dimming down through the threshold after an above-threshold
    // turn-on would spuriously re-arm kickstart mid-session.
    this->store_.was_explicitly_off = false;
  }

  this->store_.value = new_value;
}

// ── dump_config() ─────────────────────────────────────────────────────────────

void AcDimmer::dump_config() {
  ESP_LOGCONFIG(TAG,
                "AdvancedAcDimmer (esp_timer one-shot, flicker-free):\n"
                "  Min Power: %.3f\n"
                "  Max Power: %.3f\n"
                "  Init half-cycles: %u\n"
                "  Kickstart threshold: %s (%.2f)\n"
                "  Curve: %s\n"
                "  Max flat threshold: %s (%.3f)\n"
                "  Timer dispatch: %s",
                this->store_.min_power / 1000.0f,
                this->max_power_,
                this->init_with_n_half_cycles_,
                this->kickstart_threshold_ > 0.0f ? "enabled" : "disabled",
                this->kickstart_threshold_,
                this->curve_ == DIM_CURVE_RMS         ? "rms"         :
                this->curve_ == DIM_CURVE_LOGARITHMIC ? "logarithmic" : "linear",
                this->max_flat_threshold_ > 0.0f ? "enabled" : "disabled",
                this->max_flat_threshold_,
#ifdef CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD
                "ESP_TIMER_ISR (~1 µs)"
#else
                "ESP_TIMER_TASK (~10-50 µs, enable CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD for best jitter)"
#endif
  );
  LOG_PIN("  Output Pin: ", this->gate_pin_);
  LOG_PIN("  Zero-Cross Pin: ", this->zero_cross_pin_);

  const char *method_str =
      this->method_ == DIM_METHOD_LEADING_PULSE ? "leading pulse" :
      this->method_ == DIM_METHOD_LEADING        ? "leading"       : "trailing";
  ESP_LOGCONFIG(TAG, "  Dim method: %s", method_str);

  const char *zc_str =
      this->zc_method_ == ZC_METHOD_PULSE          ? "pulse (falling edge)"         :
      this->zc_method_ == ZC_METHOD_INVERTED_PULSE  ? "inverted pulse (rising edge)" :
                                                       "edges (any edge)";
  ESP_LOGCONFIG(TAG, "  ZC method: %s", zc_str);

  if (this->half_cycle_offset_us_ != 0) {
    ESP_LOGCONFIG(TAG, "  Half-cycle offset: %d µs", this->half_cycle_offset_us_);
  } else {
    ESP_LOGCONFIG(TAG, "  Half-cycle offset: disabled");
  }

  LOG_FLOAT_OUTPUT(this);

  if (this->store_.cycle_time_us > 0) {
    ESP_LOGV(TAG, "  Measured half-cycle: %" PRIu32 " µs  (%.2f Hz)",
             this->store_.cycle_time_us,
             1e6f / static_cast<float>(this->store_.cycle_time_us));
  }
}

}  // namespace esphome::advanced_ac_dimmer
