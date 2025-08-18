#pragma once

#include <atomic>
#include <cstdint>
#include <Arduino.h>

namespace wheel_hal
{
    /**
     * @brief Struct to hold encoder measurement results.
     *
     * Contains the change in position (meters) and elapsed time (seconds).
     */
    struct EncoderMeasurement
    {
        float delta_pos_m;      ///< Change in position in meters.
        float delta_time_sec; ///< Time elapsed in seconds.
    };

    /**
     * @brief Abstract base class for encoder control.
     *
     * Defines interface for setting up encoder pins and retrieving measurements.
     */
    class BaseEncoderCtrl
    {

    public:
        /**
         * @brief Set up the hardware pins required for encoder.
         */
        virtual void SetupPins() = 0;

        /**
         * @brief Get the encoder measurement since last call.
         *
         * @param is_in_reverse If true, measurement is for reverse direction.
         * @return EncoderMeasurement Struct containing position and time.
         */
        virtual EncoderMeasurement GetEncoderMeasurement(bool is_in_reverse = false) = 0;
    };

    /**
     * @brief Encoder controller for single pin encoders.
     *
     * Uses hardware interrupts to count encoder ticks. Not copyable due to
     * interrupt handler binding to instance pointer.
     */
    class SinglePinEncoderCtrl : public BaseEncoderCtrl
    {

    public:
        /**
         * @brief Construct a new SinglePinEncoderCtrl object.
         *
         * @param wheel_radius_m Wheel radius in meters.
         * @param ticks_per_rotation Encoder ticks per full wheel rotation.
         * @param encoder_pin GPIO pin connected to encoder.
         * @param pin_mode Pin mode to apply (default INPUT).
         * @param irq_mode Interrupt mode to apply (default RISING).
         */
        SinglePinEncoderCtrl(float wheel_radius_m,
                             unsigned int ticks_per_rotation,
                             uint8_t encoder_pin,
                             uint8_t pin_mode = INPUT,
                             int irq_mode = RISING);

        /**
         * @brief Destructor detaches the interrupt from the encoder pin.
         */
        ~SinglePinEncoderCtrl();

        // Not safe to copy since interrupt points to original instance.
        SinglePinEncoderCtrl(const SinglePinEncoderCtrl &) = delete;
        SinglePinEncoderCtrl(const SinglePinEncoderCtrl &&) = delete;
        SinglePinEncoderCtrl &operator=(const SinglePinEncoderCtrl &) = delete;

        /**
         * @brief Set up the encoder pin and attach interrupt handler.
         *
         * Configures the pin and reserves the hardware interrupt for counting ticks.
         *
         * Reserves the interrupt for the specified pin and binds it to this instance.
         * The interrupt handler increments the tick count atomically.
         */
        void SetupPins() override;

        /**
         * @brief Get encoder measurement since last call.
         *
         * Returns the distance traveled and time elapsed, then resets tick count.
         */
        EncoderMeasurement GetEncoderMeasurement(bool is_in_reverse = false) override;

    private:
        const uint8_t encoder_pin_;   ///< GPIO pin for encoder input.
        const uint8_t pin_mode_;      ///< Pin mode (INPUT, etc).
        const int irq_mode_;          ///< Interrupt mode (RISING, etc).
        const float ticks_to_m_ratio; ///< Conversion ratio from ticks to meters.

        volatile unsigned long encoder_ticks_ = 0; ///< Tick count, incremented by interrupt.
        unsigned long last_poll_time_us_ = 0;      ///< Last poll time in microseconds.
    };
}
