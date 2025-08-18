#pragma once

#include <cstdint>

namespace wheel_hal
{
    /**
     * @brief Abstract base class for motor control interfaces.
     *
     * Provides the interface for setting up motor control pins,
     * setting motor speed and direction.
     */
    class BaseMotorCtrl
    {

    public:
        /**
         * @brief Set up the hardware pins required for motor control.
         */
        virtual void SetupPins() = 0;

        /**
         * @brief Set the motor speed as a percentage.
         *
         * @param percent Speed percentage (0.0 to 100.0).
         * @param is_reverse If true, set motor direction to reverse.
         */
        virtual void SetSpeed(float percent, bool is_reverse = false) = 0;

        /**
         * @brief Apply brake to the motor.
         *
         * Sets speed to zero, and resists spinning if available.
         */
        virtual void Brake() { SetSpeed(0); }
    };

    /**
     * @brief Motor controller implementation for AT8236 driver.
     *
     * Controls a motor using two pins for forward and backward directions.
     */
    class AT8236MotorCtrl : public BaseMotorCtrl
    {

    public:
        /**
         * @brief Construct a new AT8236MotorCtrl object.
         *
         * @param pin_forward GPIO pin for forward direction.
         * @param pin_backward GPIO pin for backward direction.
         * @param min_active_pwm_value Minimum PWM value to activate motor.
         */
        AT8236MotorCtrl(uint8_t pin_forward,
                        uint8_t pin_backward,
                        uint8_t min_active_pwm_value = 0)
            : min_active_pwm_value_(min_active_pwm_value),
              pin_forward_(pin_forward),
              pin_backward_(pin_backward) {}

        /**
         * @brief Set up the motor control pins.
         *
         * This will set motor pins to be outputs.
         */
        void SetupPins() override;

        /**
         * @brief Set the motor speed and direction.
         *
         * @param percent Speed percentage (0.0 to 100.0).
         * @param is_reverse If true, set motor direction to reverse.
         */
        void SetSpeed(float percent, bool is_reverse = false) override;

        /**
         * @brief Apply brake to the motor.
         */
        void Brake() override;

    private:
        uint8_t min_active_pwm_value_ = 0; ///< Minimum PWM value to activate motor.
        uint8_t pin_forward_ = 0;          ///< GPIO pin for forward direction.
        uint8_t pin_backward_ = 0;         ///< GPIO pin for backward direction.

        /**
         * @brief Convert speed percentage to PWM value.
         *
         * @param percent Speed percentage (0.0 to 100.0).
         * @return uint8_t Corresponding PWM value.
         */
        uint8_t ConvertToPWMValue(float percent);
    };
}
