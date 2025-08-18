#include "encoder_ctrl.h"

#include <cmath>

#include <Arduino.h>
#include <FunctionalInterrupt.h>

using namespace wheel_hal;

// For ESP32 platforms IRQ code should be in IRAM.
#ifdef IRAM_ATTR
#define WHEEL_HAL_IRAM_ATTR IRAM_ATTR
#else
#define WHEEL_HAL_IRAM_ATTR
#endif

SinglePinEncoderCtrl::SinglePinEncoderCtrl(float wheel_radius_m,
                                           unsigned int ticks_per_rotation,
                                           uint8_t encoder_pin,
                                           uint8_t pin_mode,
                                           int irq_mode)
    : encoder_pin_(encoder_pin),
      pin_mode_(pin_mode),
      irq_mode_(irq_mode),
      ticks_to_m_ratio(wheel_radius_m * 2.0 * PI / float(ticks_per_rotation)),
      last_poll_time_us_(micros())
{
}

SinglePinEncoderCtrl::~SinglePinEncoderCtrl() {
  detachInterrupt(encoder_pin_);
}

void SinglePinEncoderCtrl::SetupPins()
{
  pinMode(this->encoder_pin_, this->pin_mode_);
  attachInterrupt(digitalPinToInterrupt(this->encoder_pin_),
                  [this]() WHEEL_HAL_IRAM_ATTR // NOLINT
                  {                            // NOLINT
                    this->encoder_ticks_++;
                  },
                  this->irq_mode_);
}

EncoderMeasurement SinglePinEncoderCtrl::GetEncoderMeasurement(bool is_in_reverse)
{
  noInterrupts();
  unsigned local_encoder_ticks = encoder_ticks_;
  encoder_ticks_ = 0;
  interrupts();

  float delta_pos_m = float(local_encoder_ticks) * ticks_to_m_ratio;

  unsigned long now_us = micros();
  float time_elapsed_sec = float(now_us - last_poll_time_us_) / 1e6;
  last_poll_time_us_ = now_us;

  return EncoderMeasurement{delta_pos_m, time_elapsed_sec};
}

#undef WHEEL_HAL_IRAM_ATTR
