#include "advanced_ac_dimmer.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <cmath>
#include <numbers>

#ifdef USE_ESP8266
#include <core_esp8266_waveform.h>
#endif

#ifdef USE_ESP32
#include "hw_timer_esp_idf.h"
#endif

namespace esphome::advanced_ac_dimmer {

static const char *const TAG = "advanced_ac_dimmer";

// Global array to store dimmer objects
static AcDimmerDataStore *all_dimmers[32];  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

/// Time in microseconds the gate should be held high
/// 10µs should be long enough for most triacs/MOSFETs
/// For reference: BT136 datasheet says 2µs nominal (page 7)
static constexpr uint32_t GATE_ENABLE_TIME = 50;

#ifdef USE_ESP32
/// Timer frequency in Hz (1 MHz = 1µs resolution)
static constexpr uint32_t TIMER_FREQUENCY_HZ = 1000000;
/// Timer interrupt interval in microseconds
static constexpr uint64_t TIMER_INTERVAL_US = 50;
#endif

/// Function called from timer interrupt
/// Input is current time in microseconds (micros())
/// Returns when next "event" is expected in µs, or 0 if no such event known.
uint32_t IRAM_ATTR HOT AcDimmerDataStore::timer_intr(uint32_t now) {
  // If no ZC signal received yet.
  if (this->crossed_zero_at == 0)
    return 0;

  uint32_t time_since_zc = now - this->crossed_zero_at;
  if (this->value == 65535 || this->value == 0) {
    return 0;
  }

  if (this->enable_time_us != 0 && time_since_zc >= this->enable_time_us) {
    this->enable_time_us = 0;
    this->gate_pin.digital_write(true);
    // Prevent too short pulses
    this->disable_time_us = std::max(this->disable_time_us, time_since_zc + GATE_ENABLE_TIME);
  }
  if (this->disable_time_us != 0 && time_since_zc >= this->disable_time_us) {
    this->disable_time_us = 0;
    this->gate_pin.digital_write(false);
  }

  if (time_since_zc < this->enable_time_us) {
    return this->enable_time_us - time_since_zc;
  } else if (time_since_zc < disable_time_us) {
    return this->disable_time_us - time_since_zc;
  }

  if (time_since_zc >= this->cycle_time_us) {
    return 100;
  }

  return this->cycle_time_us - time_since_zc;
}

/// Run timer interrupt code and return in how many µs the next event is expected
uint32_t IRAM_ATTR HOT timer_interrupt() {
  // run at least with 1kHz
  uint32_t min_dt_us = 1000;
  uint32_t now = micros();
  for (auto *dimmer : all_dimmers) {
    if (dimmer == nullptr) {
      break;
    }
    uint32_t res = dimmer->timer_intr(now);
    if (res != 0 && res < min_dt_us)
      min_dt_us = res;
  }
  return min_dt_us;
}

/// GPIO interrupt routine, called on both edges of the ZC signal.
/// Both rising and falling edges represent zero crossings when the ZCD circuit
/// outputs a sustained level (0V on positive half-cycle, 3.3V on negative half-cycle)
/// rather than a narrow pulse. INTERRUPT_ANY_EDGE is set in setup() to ensure both
/// zero crossings per mains cycle are detected, giving correct half-cycle timing.
void IRAM_ATTR HOT AcDimmerDataStore::gpio_intr() {
  uint32_t prev_crossed = this->crossed_zero_at;

  // At 60Hz a half-cycle is 8.33ms; at 50Hz it is 10ms.
  // The noise filter threshold of 5ms rejects spurious edges within the same pulse.
  this->crossed_zero_at = micros();
  uint32_t cycle_time = this->crossed_zero_at - prev_crossed;
  if (cycle_time > 5000) {
    this->cycle_time_us = cycle_time;
  } else {
    this->cycle_time_us += cycle_time;
  }

  if (this->value == 65535) {
    // fully on, enable output immediately
    this->gate_pin.digital_write(true);
  } else if (this->init_cycle_count > 0) {
    // Kickstart: drive gate high immediately and hold for the full half-cycle.
    // enable_time_us must NOT be set to 0 here — 0 means "not set" in timer_intr
    // and the gate would never be enabled. Drive it directly instead, same as the
    // value==65535 path, then let disable_time_us turn it off at end of half-cycle.
    this->init_cycle_count--;
    this->gate_pin.digital_write(true);
    this->enable_time_us = 0;            // already on — no timer enable needed
    this->disable_time_us = cycle_time_us;
  } else if (this->value == 0) {
    // fully off, disable output immediately
    this->gate_pin.digital_write(false);
  } else {
    auto min_us = this->cycle_time_us * this->min_power / 1000;
    if (this->method == DIM_METHOD_TRAILING) {
      this->enable_time_us = 1;  // cannot be 0
      this->disable_time_us = std::max((uint32_t) 10, this->value * (this->cycle_time_us - min_us) / 65535 + min_us);
    } else {
      this->enable_time_us = std::max((uint32_t) 1, ((65535 - this->value) * (this->cycle_time_us - min_us)) / 65535);

      if (this->method == DIM_METHOD_LEADING_PULSE) {
        this->disable_time_us = std::max(this->enable_time_us + GATE_ENABLE_TIME, (uint32_t) cycle_time_us / 10);
      } else {
        this->gate_pin.digital_write(false);
        this->disable_time_us = this->cycle_time_us;
      }
    }
  }
}

void IRAM_ATTR HOT AcDimmerDataStore::s_gpio_intr(AcDimmerDataStore *store) {
  // When multiple dimmers share the same ZC pin, trigger all of them.
  for (auto *dimmer : all_dimmers) {
    if (dimmer == nullptr)
      break;
    if (dimmer->zero_cross_pin_number == store->zero_cross_pin_number) {
      dimmer->gpio_intr();
    }
  }
}

#ifdef USE_ESP32
static HWTimer *dimmer_timer = nullptr;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
void IRAM_ATTR HOT AcDimmerDataStore::s_timer_intr() { timer_interrupt(); }
#endif

void AcDimmer::setup() {
  auto setup_zero_cross_pin = true;

  for (auto &all_dimmer : all_dimmers) {
    if (all_dimmer == nullptr) {
      all_dimmer = &this->store_;
      break;
    }
    if (all_dimmer->zero_cross_pin_number == this->zero_cross_pin_->get_pin()) {
      setup_zero_cross_pin = false;
    }
  }

  this->gate_pin_->setup();
  this->store_.gate_pin = this->gate_pin_->to_isr();
  this->store_.zero_cross_pin_number = this->zero_cross_pin_->get_pin();
  this->store_.min_power = static_cast<uint16_t>(this->min_power_ * 1000);
  this->min_power_ = 0;
  this->store_.method = this->method_;

  if (setup_zero_cross_pin) {
    this->zero_cross_pin_->setup();
    this->store_.zero_cross_pin = this->zero_cross_pin_->to_isr();
    // Select interrupt mode based on configured zc_method:
    //   edges          → ANY_EDGE:     sustained-level ZCD (H11A1-based). Both edges = ZC.
    //   pulse          → FALLING_EDGE: active-low narrow pulse (upstream ac_dimmer default).
    //   inverted_pulse → RISING_EDGE:  active-high narrow pulse.
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
    this->zero_cross_pin_->attach_interrupt(&AcDimmerDataStore::s_gpio_intr, &this->store_,
                                            intr_type);
  }

#ifdef USE_ESP8266
  setTimer1Callback(&timer_interrupt);
#endif
#ifdef USE_ESP32
  if (dimmer_timer == nullptr) {
    dimmer_timer = timer_begin(TIMER_FREQUENCY_HZ);
    if (dimmer_timer == nullptr) {
      ESP_LOGE(TAG, "Failed to create GPTimer for AC dimmer");
      this->mark_failed();
      return;
    }
    timer_attach_interrupt(dimmer_timer, &AcDimmerDataStore::s_timer_intr);
    timer_alarm(dimmer_timer, TIMER_INTERVAL_US, true, 0);
  }
#endif
}

void AcDimmer::write_state(float state) {
  // State arrives here after FloatOutput remaps it: state = min_power + raw*(max_power-min_power).
  // With gamma_correct: 0 (or 1) on the light entity and typical min_power=0.005, max_power=1.0,
  // state at slider=99% ≈ 0.990 and state at slider=100% == max_power_ (1.0) exactly.
  //
  // GAMMA NOTE: the acos RMS compensation below corrects the nonlinear relationship between
  // conduction angle and RMS power — it is NOT gamma correction. Gamma correction (applied
  // by the light component before calling write_state) shifts the state into a different space:
  // e.g. gamma=2.8 maps slider 90% → state 0.73, so threshold 0.9 is never reached and
  // threshold 0.5 triggers at slider ~78% instead of 50%.
  // Set gamma_correct: 0 or gamma_correct: 1 on the light entity to disable gamma and make
  // threshold comparisons match the HA slider position directly.
  //
  // RMS CORRECTION NOTE (rms_correction option):
  // The acos transform corrects for the nonlinear relationship between phase-angle conduction
  // and RMS power, which is appropriate for resistive/incandescent loads (brightness ∝ power).
  // For LED lamps with constant-current switching drivers, brightness is proportional to
  // conduction fraction (linear) — acos over-corrects and makes dimming less accurate.
  // Set rms_correction: false for LED loads.

  uint16_t new_value;

  auto apply_compensation = [&](float s) -> uint16_t {
    if (this->rms_correction_) {
      s = std::acos(1 - (2 * s)) / std::numbers::pi;
    }
    return static_cast<uint16_t>(roundf(s * 65535));
  };

  if (state == 0.0f) {
    // Fully off.
    new_value = 0;

  } else if (this->max_flat_threshold_ > 0.0f) {
    // Flat zone mode: the dimmable range [min_power, max_flat_threshold] is compressed
    // to fill slider positions 1–99%, and slider 100% (state == max_power_) jumps to
    // max_power (full conduction). This gives the UI a full 1–100% range regardless of
    // where max_flat_threshold is set.
    //
    // FloatOutput guarantees state == max_power_ at slider=100% (since
    // state = min_power + 1.0*(max_power-min_power) = max_power). A small epsilon
    // guards against float rounding in the remapping below.
    if (state >= this->max_power_ - 1e-5f) {
      // Slider at 100%: jump to max_power.
      // max_power_ == 1.0 → 65535 → gpio_intr holds gate high for the full half-cycle,
      // zero switching losses. Otherwise apply RMS compensation to the capped level.
      if (this->max_power_ >= 1.0f) {
        new_value = 65535;
      } else {
        new_value = apply_compensation(this->max_power_);
      }
    } else {
      // Slider 1–99%: remap state from [0, max_power_] to [0, max_flat_threshold_] so
      // the entire dimmable range spans the slider linearly.
      float remapped = (state / this->max_power_) * this->max_flat_threshold_;
      new_value = apply_compensation(remapped);
    }

  } else {
    // No flat zone: standard dimming across full [0, max_power_] range.
    new_value = apply_compensation(state);
  }

  if (new_value != 0 && this->store_.value == 0)
    this->store_.init_cycle_count = this->init_with_n_half_cycles_;
  this->store_.value = new_value;
}

void AcDimmer::dump_config() {
  ESP_LOGCONFIG(TAG,
                "EdgeAcDimmer:\n"
                "   Min Power: %.3f\n"
                "   Max Power: %.3f\n"
                "   Init half-cycles: %u\n"
                "   RMS correction: %s\n"
                "   Max flat threshold: %s (%.3f)",
                this->store_.min_power / 1000.0f,
                this->max_power_,
                this->init_with_n_half_cycles_,
                YESNO(this->rms_correction_),
                this->max_flat_threshold_ > 0.0f ? "enabled" : "disabled",
                this->max_flat_threshold_);
  LOG_PIN("  Output Pin: ", this->gate_pin_);
  LOG_PIN("  Zero-Cross Pin: ", this->zero_cross_pin_);
  if (method_ == DIM_METHOD_LEADING_PULSE) {
    ESP_LOGCONFIG(TAG, "   Dim method: leading pulse");
  } else if (method_ == DIM_METHOD_LEADING) {
    ESP_LOGCONFIG(TAG, "   Dim method: leading");
  } else {
    ESP_LOGCONFIG(TAG, "   Dim method: trailing");
  }
  if (zc_method_ == ZC_METHOD_PULSE) {
    ESP_LOGCONFIG(TAG, "   ZC method: pulse (falling edge)");
  } else if (zc_method_ == ZC_METHOD_INVERTED_PULSE) {
    ESP_LOGCONFIG(TAG, "   ZC method: inverted pulse (rising edge)");
  } else {
    ESP_LOGCONFIG(TAG, "   ZC method: edges (any edge)");
  }
  LOG_FLOAT_OUTPUT(this);
  ESP_LOGV(TAG, "  Estimated Frequency: %.3fHz", 1e6f / this->store_.cycle_time_us / 2);
}

}  // namespace esphome::advanced_ac_dimmer
