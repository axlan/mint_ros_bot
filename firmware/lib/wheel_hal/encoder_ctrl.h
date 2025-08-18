#pragma once

#include <atomic>
#include <cstdint>

#include <Arduino.h>

namespace wheel_hal
{
    struct EncoderMeasurement
    {
        float delta_pos_m;
        float time_elapsed_sec;
    };

    class BaseEncoderCtrl
    {

    public:
        virtual void SetupPins() = 0;
        virtual EncoderMeasurement GetEncoderMeasurement(bool is_in_reverse = false) = 0;
    };

    class SinglePinEncoderCtrl : public BaseEncoderCtrl
    {

    public:
        SinglePinEncoderCtrl(float wheel_radius_m,
                             unsigned int ticks_per_rotation,
                             uint8_t encoder_pin,
                             uint8_t pin_mode = INPUT,
                             int irq_mode = RISING);
        ~SinglePinEncoderCtrl();
        // Not safe to copy since interrupt points to original instance.
        SinglePinEncoderCtrl(const SinglePinEncoderCtrl &) = delete;
        SinglePinEncoderCtrl(const SinglePinEncoderCtrl &&) = delete;
        SinglePinEncoderCtrl &operator=(const SinglePinEncoderCtrl &) = delete;
        void SetupPins() override;
        EncoderMeasurement GetEncoderMeasurement(bool is_in_reverse = false) override;

    private:
        const uint8_t encoder_pin_;
        const uint8_t pin_mode_;
        const int irq_mode_;
        const float ticks_to_m_ratio;

        volatile unsigned long encoder_ticks_ = 0;
        unsigned long last_poll_time_us_ = 0;
    };
}
