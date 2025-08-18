#include <Arduino.h>

#include <odometry.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/timer.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>

// Combines
// www.hackster.io/amal-shaji/differential-drive-robot-using-ros2-and-esp32-aae289
// https://github.com/micro-ROS/micro_ros_platformio/blob/main/examples/ethernet_pubsub/src/main.cpp

////// Pin declarations

static constexpr int8_t PIN_LED = 2;

//// Left wheel
// Control pins need to be able to generate PWM
static constexpr int8_t PIN_L_FORW = 5;
static constexpr int8_t PIN_L_BACK = 18;

//// Right wheel
// Control pins need to be able to generate PWM
static constexpr int8_t PIN_R_FORW = 19;
static constexpr int8_t PIN_R_BACK = 21;

void setup()
{
  Serial.begin(115200);
  Serial.println("[INIT] Starting micro-ROS node...");

  pinMode(PIN_L_FORW, OUTPUT);
  pinMode(PIN_L_BACK, OUTPUT);

  pinMode(PIN_R_FORW, OUTPUT);
  pinMode(PIN_R_BACK, OUTPUT);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
}

void loop()
{
  analogWrite(PIN_L_FORW, 0);
  analogWrite(PIN_L_BACK, 200);
  analogWrite(PIN_R_FORW, 0);
  analogWrite(PIN_R_BACK, 200);

  delay(5000);

  analogWrite(PIN_L_BACK, 0);
  analogWrite(PIN_L_FORW, 200);
  analogWrite(PIN_R_BACK, 0);
  analogWrite(PIN_R_FORW, 200);

  delay(5000);
}
