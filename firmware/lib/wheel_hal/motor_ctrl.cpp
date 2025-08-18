#include "motor_ctrl.h"

#include <Arduino.h>

#include <cmath>

using namespace wheel_hal;

uint8_t AT8236MotorCtrl::ConvertToPWMValue(float percent)
{
    if (percent == 0)
    {
        return 0;
    }
    else
    {
        float abs_ratio = std::abs(percent) / 100.0;
        float range = (255 - min_active_pwm_value_);
        return static_cast<uint8_t>(std::round(range * abs_ratio) + min_active_pwm_value_);
    }
}

void AT8236MotorCtrl::SetupPins()
{
    pinMode(this->pin_forward_, OUTPUT);
    pinMode(this->pin_backward_, OUTPUT);
}

void AT8236MotorCtrl::SetSpeed(float percent, bool is_reverse)
{
    uint8_t pwm_pin = (is_reverse) ? this->pin_backward_ : this->pin_forward_;
    uint8_t ctrl_pin = (is_reverse) ? this->pin_forward_ : this->pin_backward_;
    // For ESP32, recent version of analogWrite (since 2021) wrap the lower level ledc library.
    analogWrite(ctrl_pin, 0);
    analogWrite(pwm_pin, ConvertToPWMValue(percent));
}

void AT8236MotorCtrl::Brake()
{
    analogWrite(this->pin_forward_, 255);
    analogWrite(this->pin_backward_, 255);
}
