#pragma once

#include <cstdint>

namespace wheel_hal
{
    class BaseMotorCtrl
    {

    public:
        virtual void SetupPins() = 0;
        virtual void SetSpeed(float percent, bool is_reverse = false) = 0;
        virtual void Brake() { SetSpeed(0); }
    };

    class AT8236MotorCtrl : public BaseMotorCtrl
    {

    public:
        AT8236MotorCtrl(uint8_t pin_forward,
                        uint8_t pin_backward,
                        uint8_t min_active_pwm_value = 0)
            : min_active_pwm_value_(min_active_pwm_value),
              pin_forward_(pin_forward),
              pin_backward_(pin_backward) {}

        void SetupPins() override;
        void SetSpeed(float percent, bool is_reverse = false) override;
        void Brake() override;

    private:
        uint8_t min_active_pwm_value_ = 0;
        uint8_t pin_forward_ = 0;
        uint8_t pin_backward_ = 0;

        uint8_t ConvertToPWMValue(float percent);
    };
}
