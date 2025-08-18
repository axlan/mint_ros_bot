
#include "motor_ctrl.h"
#include "encoder_ctrl.h"

// Combines
// www.hackster.io/amal-shaji/differential-drive-robot-using-ros2-and-esp32-aae289
// https://github.com/micro-ROS/micro_ros_platformio/blob/main/examples/ethernet_pubsub/src/main.cpp

////// Pin declarations

static constexpr int8_t PIN_LED = 2;

//// Left wheel
// Control pins need to be able to generate PWM
static constexpr int8_t PIN_R_FORW = 18;
static constexpr int8_t PIN_R_BACK = 5;
// Encoder needs to support interrupts.
static constexpr int8_t PIN_R_ENCODER = 27;

//// Right wheel
// Control pins need to be able to generate PWM
static constexpr int8_t PIN_L_FORW = 21;
static constexpr int8_t PIN_L_BACK = 19;
// Encoder needs to support interrupts.
static constexpr int8_t PIN_L_ENCODER = 26;

////// Parameters of the robot
// Units in meters
static constexpr float WHEELS_Y_DISTANCE = 0.173;
static constexpr float WHEEL_RADIUS = 0.0245;
static constexpr float WHEEL_CIRCUMFERENCE = 0.153;
//// Encoder value per revolution of left wheel and right wheel
static constexpr int TICK_PER_REVOLUTION_LW = 4000;
static constexpr int TICK_PER_REVOLUTION_RW = 4000;
// Min value for PWM that moves wheels
static constexpr int PWM_THRESHOLD = 100;
//// PID constants of left wheel
static constexpr float PID_KP_L = 1.8;
static constexpr float PID_KI_L = 5;
static constexpr float PID_KD_L = 0.1;
//// PID constants of right wheel
static constexpr float PID_KP_R = 1.8;
static constexpr float PID_KI_R = 5;
static constexpr float PID_KD_R = 0.1;

//// pwm parameters setup
static constexpr int PWM_FREQ = 30000;
static constexpr int PWM_CHANNEL_L_FORW = 0;
static constexpr int PWM_CHANNEL_L_BACK = 1;
static constexpr int PWM_CHANNEL_R_FORW = 2;
static constexpr int PWM_CHANNEL_R_BACK = 3;
static constexpr int PWM_RESOLUTION = 8;

using namespace wheel_hal;

SinglePinEncoderCtrl left_encoder(WHEEL_RADIUS, TICK_PER_REVOLUTION_LW, PIN_L_ENCODER, INPUT, FALLING);
SinglePinEncoderCtrl right_encoder(WHEEL_RADIUS, TICK_PER_REVOLUTION_RW, PIN_R_ENCODER, INPUT, FALLING);

AT8236MotorCtrl left_motor(PIN_L_FORW, PIN_L_BACK, PWM_THRESHOLD);
AT8236MotorCtrl right_motor(PIN_R_FORW, PIN_R_BACK, PWM_THRESHOLD);

void setup()
{
  Serial.begin(115200);
  Serial.println("[INIT] Starting micro-ROS node...");

  left_motor.SetupPins();
  right_motor.SetupPins();

  left_encoder.SetupPins();
  right_encoder.SetupPins();
}

void loop()
{
  Serial.println("Left");
  auto left = left_encoder.GetEncoderMeasurement();
  Serial.println(left.delta_pos_m * 1000.0);
  Serial.println(left.time_elapsed_sec);

  Serial.println("Right");
  auto right = right_encoder.GetEncoderMeasurement();
  Serial.println(right.delta_pos_m * 1000.0);
  Serial.println(right.time_elapsed_sec);

  Serial.println();

  left_motor.SetSpeed(50, false);
  //right_motor.SetSpeed(50, true);
  delay(1000);

  left_motor.SetSpeed(0);
  right_motor.SetSpeed(0);
  delay(4000);


}
