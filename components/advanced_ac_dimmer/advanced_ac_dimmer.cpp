#include "advanced_ac_dimmer.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cinttypes>
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

// ── Diagnostic counters (temporary — for tracking down intermittent flicker) ──
// Incremented from s_gpio_intr() (GPIO ISR) once per physical edge.
// Read and reset from AcDimmer::loop() (main-loop context).
// A benign, undocumented race between an ISR increment and the main-loop
// read/reset can drop at most one count per report interval, which is fine
// for a diagnostic counter — never used for control logic.
static DRAM_ATTR volatile uint32_t zc_accepted_count = 0;
static DRAM_ATTR volatile uint32_t zc_rejected_count = 0;

// ── Constants ─────────────────────────────────────────────────────────────────

/// Valid mains half-cycle band (covers 45–67 Hz with ZCD asymmetry).
static constexpr uint32_t ZC_PERIOD_MIN_US = 7500;
static constexpr uint32_t ZC_PERIOD_MAX_US = 11000;

/// While unlocked, ignore gaps shorter than this so a mid-cycle glitch cannot
/// become the PLL origin. 6000 µs rejects 90°-point noise at 50/60 Hz.
static constexpr uint32_t ZC_UNLOCKED_MIN_GAP_US = 6000;

/// (added) Lost-lock recovery. If no edge has been accepted for this many periods
/// (mains dropout, or a phase step beyond the gate below) the lock is dropped and
/// re-acquired on the next edge. Without this, once the PLL falls more than ~5
/// half-cycles behind it can never accept an edge again (output stays dead until reboot).
static constexpr int64_t ZC_RELOCK_PERIODS = 8;

/// (added) Capture gate once locked. The +-25 % test below is far wider than real ZCD
/// jitter, so a noise edge 1-2 ms before the real one was accepted and dragged the
/// phase by hundreds of microseconds for ~10 half-cycles.
static constexpr int64_t ZC_LOCK_WINDOW_US = 300;

/// (added) Largest error the loop follows in a single edge. A real phase step is still
/// tracked (60/4 = 15 us per edge); an accepted outlier can move the phase by at most that.
static constexpr int64_t ZC_ERR_CLIP_US = 60;

/// (added) ZC pin level sampled as the FIRST thing in the ISR and read by gpio_intr(),
/// instead of reading the pin after the timers were stopped (which can land on chatter
/// and apply half_cycle_offset to the wrong half-cycle).
static DRAM_ATTR volatile bool zc_level_isr = false;

/// (added) First of the two intervals averaged during acquisition (0 = none yet).
static DRAM_ATTR uint32_t zc_acq_first = 0;

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

// ── Zero-crossing PLL ─────────────────────────────────────────────────────────
//
// last_zc_time is a virtual ZC on a slowly-tracked mains grid, not the raw GPIO
// timestamp. Gate one-shots are then armed as
//   delay = (last_zc_time + phase_us) − now
// so a late GPIO ISR shortens the delay instead of shifting the firing angle.
//
// Edges that are not near the predicted instant are ignored *before* Pass 1, so
// a mid-cycle ZCD glitch cannot abort an in-flight half-cycle (the classic
// burst / flicker failure mode of a 3 ms debounce).

static bool IRAM_ATTR HOT lock_zero_cross(AcDimmerDataStore *store, int64_t now) {
  if (store->last_zc_time == 0) {
    store->last_zc_time = now;
    return true;  // first edge: origin only, no period yet
  }

  const int64_t elapsed = now - store->last_zc_time;

  if (store->cycle_time_us == 0) {
    if (elapsed < static_cast<int64_t>(ZC_PERIOD_MIN_US) ||
        elapsed > static_cast<int64_t>(ZC_PERIOD_MAX_US)) {
      if (elapsed >= static_cast<int64_t>(ZC_UNLOCKED_MIN_GAP_US)) {
        store->last_zc_time = now;
      }
      zc_acq_first = 0;   // (added) sequence broken, start over
      return false;
    }
    // (added) The raw interval is one of two alternating values (the ZCD detects its two edges
    // at different delays), so a single interval is off by tens of microseconds and the loop
    // then needs ~16 half-cycles to settle (up to +70 % light at the lowest levels). Averaging
    // two consecutive intervals cancels that alternation and gives the period directly.
    if (zc_acq_first == 0) {
      zc_acq_first = static_cast<uint32_t>(elapsed);
      store->last_zc_time = now;
      return true;        // still unlocked (cycle_time_us == 0): s_gpio_intr arms nothing yet
    }
    store->cycle_time_us = (zc_acq_first + static_cast<uint32_t>(elapsed)) / 2;
    zc_acq_first = 0;
    store->last_zc_time = now;
    return true;
  }

  const uint32_t period = store->cycle_time_us;
  if (elapsed > static_cast<int64_t>(period) * ZC_RELOCK_PERIODS) {
    store->cycle_time_us = 0;      // drop the lock ...
    zc_acq_first = 0;
    store->last_zc_time = now;     // ... and make this edge the new origin
    return true;
  }
  int64_t expected = store->last_zc_time + static_cast<int64_t>(period);
  int missed = 0;
  while ((now - expected) > static_cast<int64_t>(period / 2) && missed < 4) {
    expected += period;
    missed++;
  }

  const int64_t error = now - expected;
  const int64_t abs_error = (error < 0) ? -error : error;
  const int64_t window = static_cast<int64_t>(period / 4);  // ±25 %
  if (abs_error > window) {
    return false;
  }
  if (abs_error > ZC_LOCK_WINDOW_US) {   // (added) stricter than the +-25 % test above
    return false;
  }
  const int64_t err_c = (error > ZC_ERR_CLIP_US) ? ZC_ERR_CLIP_US : ((error < -ZC_ERR_CLIP_US) ? -ZC_ERR_CLIP_US : error);

  int32_t new_period = static_cast<int32_t>(period) + static_cast<int32_t>(err_c / 32);
  if (new_period < static_cast<int32_t>(ZC_PERIOD_MIN_US)) {
    new_period = static_cast<int32_t>(ZC_PERIOD_MIN_US);
  } else if (new_period > static_cast<int32_t>(ZC_PERIOD_MAX_US)) {
    new_period = static_cast<int32_t>(ZC_PERIOD_MAX_US);
  }
  store->cycle_time_us = static_cast<uint32_t>(new_period);
  // Pull phase 1/4 of the way toward the measurement so ISR jitter is filtered
  // but a slow mains drift is still tracked.
  store->last_zc_time = expected + (err_c / 4);
  return true;
}

#ifdef USE_ESP32
static void IRAM_ATTR HOT arm_timer_at(esp_timer_handle_t timer, int64_t virtual_zc, int32_t phase_us) {
  int64_t delay = (virtual_zc + phase_us) - esp_timer_get_time();
  if (delay < 1) {
    delay = 1;
  }
  esp_timer_start_once(timer, static_cast<uint64_t>(delay));
}
#endif

/// gpio_intr: called by s_gpio_intr (pass 2) after timers have been stopped.
/// last_zc_time / cycle_time_us are already the shared PLL state for this edge.
void IRAM_ATTR HOT AcDimmerDataStore::gpio_intr() {
#ifndef USE_ESP32
  return;
#else
  if (this->value == 65535) {
    this->gate_pin.digital_write(true);
    return;
  }

  if (this->init_cycle_count > 0) {
    if (this->kickstart_threshold_value > 0 && this->value >= this->kickstart_threshold_value) {
      this->init_cycle_count = 0;
    } else {
      this->init_cycle_count--;
      this->gate_pin.digital_write(true);
      if (this->cycle_time_us > 0) {
        arm_timer_at(this->disable_timer, this->last_zc_time, static_cast<int32_t>(this->cycle_time_us));
      }
      return;
    }
  }

  if (this->value == 0 || this->cycle_time_us == 0) {
    return;
  }

  const uint32_t min_us = this->cycle_time_us * this->min_power / 1000;
  const uint32_t span = this->cycle_time_us - min_us;
  bool apply_offset = false;

  if (this->half_cycle_offset_us != 0) {
    if (this->zc_method == ZC_METHOD_EDGES) {
      apply_offset = !zc_level_isr;
    } else {
      this->half_cycle_toggle = !this->half_cycle_toggle;
      apply_offset = this->half_cycle_toggle;
    }
  }

  if (this->method == DIM_METHOD_TRAILING) {
    int32_t adj = static_cast<int32_t>(this->value * span / 65535 + min_us);
    if (apply_offset) {
      adj += this->half_cycle_offset_us;
    }
    if (adj < static_cast<int32_t>(GATE_ENABLE_TIME + 1)) {
      adj = static_cast<int32_t>(GATE_ENABLE_TIME + 1);
    }
    this->gate_pin.digital_write(true);
    arm_timer_at(this->disable_timer, this->last_zc_time, adj);
  } else {
    int32_t adj = static_cast<int32_t>(((65535 - this->value) * span) / 65535);
    if (adj < 1) {
      adj = 1;
    }
    if (apply_offset) {
      adj -= this->half_cycle_offset_us;
    }
    if (adj < 1) {
      adj = 1;
    }
    if (this->method == DIM_METHOD_LEADING_PULSE) {
      this->pending_disable_us = GATE_ENABLE_TIME;
    }
    arm_timer_at(this->enable_timer, this->last_zc_time, adj);
  }
#endif
}

/// GPIO ISR entry point.
///
/// Reject glitches first. Only an accepted ZC runs the two-pass protocol:
///   Pass 1: stop timers; gate LOW unless fully on / kickstart.
///   Pass 2: arm one-shots from the shared virtual ZC.
void IRAM_ATTR HOT AcDimmerDataStore::s_gpio_intr(AcDimmerDataStore *store) {
#ifdef USE_ESP32
  const int64_t now = esp_timer_get_time();
  zc_level_isr = store->zero_cross_pin.digital_read();  // (added) sample the level first, before anything else
  if (!lock_zero_cross(store, now)) {
    zc_rejected_count++;
    return;
  }
  zc_accepted_count++;

  const uint32_t period = store->cycle_time_us;
  if (period == 0) {
    return;  // first edge: PLL origin only
  }

  const uint8_t pin = store->zero_cross_pin_number;
  const int64_t virtual_zc = store->last_zc_time;

  for (auto *dimmer : all_dimmers) {
    if (dimmer == nullptr)
      break;
    if (dimmer->zero_cross_pin_number != pin)
      continue;
    dimmer->last_zc_time = virtual_zc;
    dimmer->cycle_time_us = period;
    esp_timer_stop(dimmer->enable_timer);
    esp_timer_stop(dimmer->disable_timer);
    if (dimmer->value != 65535 && dimmer->init_cycle_count == 0) {
      dimmer->gate_pin.digital_write(false);
    }
  }
  for (auto *dimmer : all_dimmers) {
    if (dimmer == nullptr)
      break;
    if (dimmer->zero_cross_pin_number == pin) {
      dimmer->gpio_intr();
    }
  }
#else
  for (auto *dimmer : all_dimmers) {
    if (dimmer == nullptr)
      break;
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

// ── loop() ─────────────────────────────────────────────────────────────────────
// Temporary diagnostic: periodically reports the ZC accept/reject rate to help
// correlate flickering episodes with spurious zero-crossing edges (e.g. grid
// noise coupling into the ZCD). Remove once the investigation concludes.
//
// Only one AcDimmer instance's loop() call actually reports per interval.
// ESPHome runs every component's loop() sequentially on one task each pass,
// so whichever instance's loop() first crosses REPORT_INTERVAL_MS resets the
// shared timer; any other instance's loop() call later in that same pass then
// sees an elapsed time near zero and skips. No per-instance "owner" flag needed.
void AcDimmer::loop() {
  static uint32_t last_report_ms = 0;
  static constexpr uint32_t REPORT_INTERVAL_MS = 5000;

  uint32_t now_ms = millis();
  if (now_ms - last_report_ms < REPORT_INTERVAL_MS) {
    return;
  }
  last_report_ms = now_ms;

  uint32_t accepted = zc_accepted_count;
  uint32_t rejected = zc_rejected_count;
  zc_accepted_count = 0;
  zc_rejected_count = 0;

  ESP_LOGD(TAG,
           "ZC diag: accepted=%" PRIu32 " rejected=%" PRIu32 " over %" PRIu32
           " ms  half-cycle=%" PRIu32 " us  freq=%.2f Hz",
           accepted, rejected, REPORT_INTERVAL_MS, this->store_.cycle_time_us,
           this->get_frequency_hz());
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
             this->store_.cycle_time_us, this->get_frequency_hz());
  }
}

}  // namespace esphome::advanced_ac_dimmer
