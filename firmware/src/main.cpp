#include <Arduino.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/timer.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>

#include <odometry.h>

#include "motor_ctrl.h"
#include "encoder_ctrl.h"

// untracked file with:
// char WIFI_SSID[] = "";
// char WIFI_PASSWORD[] = "";
// For some reason these need to be passed as mutable to set_microros_wifi_transports?
#include "secrets.h"

using namespace wheel_hal;

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
////// Parameters of the robot
// Units in meters
static constexpr float WHEELS_Y_DISTANCE = .173;
static constexpr float WHEEL_RADIUS = 0.0245;
static constexpr float WHEEL_CIRCUMFERENCE = 0.153;
//// Encoder value per revolution of left wheel and right wheel
static constexpr int TICK_PER_REVOLUTION = 4000;
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

////// Network configuration
static const IPAddress AGENT_IP(192, 168, 1, 115);
static const uint16_t AGENT_PORT = 8888;

////// ROS node configuration
static constexpr const char *ROS_NODE_NAME = "franken_mint_node";
static constexpr int ROS_EXECUTOR_TIMEOUT = 100; // ms

// Update controller at 10Hz.
static constexpr unsigned int CONTROL_TIMER_PERIOD_MS = 100;

// creating a class for motor control
class PIDFilter
{
public:
  PIDFilter(float proportionalGain, float integralGain, float derivativeGain) : kp(proportionalGain), ki(integralGain), kd(derivativeGain)
  {
  }

  float Update(float setpoint, float feedback, float time_delta_sec)
  {
    float error = setpoint - feedback;
    float eintegral = eintegral + (error * time_delta_sec);
    float ederivative = (error - previous_error) / time_delta_sec;
    float control_signal = (kp * error) + (ki * eintegral) + (kd * ederivative);

    previous_error = error;
    return control_signal;
  }

private:
  const float kp;
  const float ki;
  const float kd;
  float previous_error = 0;
};

/**
 * @brief Class for controlling brushed motor speed.
 *
 * This is fairly specific for this differential wheel PID controller. To make
 * it more general, the interface may need to be rethought. Also, the ownership
 * and types of class components (motor/encoder/filter/etc.) would probably need
 * to be reworked.
 */
class PIDController
{
public:
  // For managing the lifetime of the objects, these should probably be unique
  // pointers. Leaving them as bare pointers for simplicity.
  PIDController(BaseMotorCtrl *motor_ctrl,
                BaseEncoderCtrl *encoder_ctrl,
                PIDFilter *pid_filter) : motor_ctrl_(motor_ctrl), encoder_ctrl_(encoder_ctrl), pid_filter_(pid_filter) {}

  void Setup()
  {
    motor_ctrl_->SetupPins();
    encoder_ctrl_->SetupPins();
  }

  EncoderMeasurement Update(float cmd_velocity_mps)
  {
    // For single pin encoders:
    // If the new command reverses the motor direction, any encoder ticks that
    // occur between the measurement and the new command will be counted
    // incorrectly.
    EncoderMeasurement encoder_meas = encoder_ctrl_->GetEncoderMeasurement(is_in_reverse_);
    float meas_vel_mps = encoder_meas.delta_pos_m / encoder_meas.delta_time_sec;

    float actuating_signal = 0;
    if (cmd_velocity_mps != 0)
    {
      actuating_signal = pid_filter_->Update(cmd_velocity_mps, meas_vel_mps, encoder_meas.delta_time_sec);
    }

    is_in_reverse_ = actuating_signal < 0;
    motor_ctrl_->SetSpeed(abs(actuating_signal), is_in_reverse_);

    return encoder_meas;
  }

private:
  BaseMotorCtrl *motor_ctrl_ = nullptr;
  BaseEncoderCtrl *encoder_ctrl_ = nullptr;
  PIDFilter *pid_filter_ = nullptr;
  bool is_in_reverse_ = false;
};

////// Global variables

// Motor interface
SinglePinEncoderCtrl left_encoder(WHEEL_RADIUS, TICK_PER_REVOLUTION, PIN_L_ENCODER, INPUT, FALLING);
SinglePinEncoderCtrl right_encoder(WHEEL_RADIUS, TICK_PER_REVOLUTION, PIN_R_ENCODER, INPUT, FALLING);
AT8236MotorCtrl left_motor(PIN_L_FORW, PIN_L_BACK, PWM_THRESHOLD);
AT8236MotorCtrl right_motor(PIN_R_FORW, PIN_R_BACK, PWM_THRESHOLD);

// PID interface
PIDFilter left_wheel_pid(PID_KP_L, PID_KI_L, PID_KD_L);
PIDFilter right_wheel_pid(PID_KP_R, PID_KI_R, PID_KD_R);

PIDController left_ctrl(&right_motor, &right_encoder, &right_wheel_pid);
PIDController right_ctrl(&right_motor, &right_encoder, &right_wheel_pid);

// ROS entities
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rcl_timer_t control_timer;

// Message buffers
geometry_msgs__msg__Twist cmd_msg;
nav_msgs__msg__Odometry odom_msg;

unsigned long long time_offset = 0;
unsigned long prev_odom_update = 0;
Odometry odometry;

// Connection management
enum class ConnectionState
{
  kInitializing,
  kWaitingForAgent,
  kConnecting,
  kConnected,
  kDisconnected
};
ConnectionState connection_state = ConnectionState::kInitializing;

struct timespec getTime()
{
  struct timespec tp = {0};
  // add time difference between uC time and ROS time to
  // synchronize time with ROS
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;
  return tp;
}

// function which publishes wheel odometry.
void publishData()
{
  odom_msg = odometry.getData();

  struct timespec time_stamp = getTime();

  odom_msg.header.stamp.sec = time_stamp.tv_sec;
  odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  if (rcl_publish(&publisher, &odom_msg, NULL) != RCL_RET_OK)
  {
    Serial.println("[PUB] Failed");
  }
}

void MotorControllerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  float linearVelocity;
  float angularVelocity;
  // linear velocity and angular velocity send cmd_vel topic
  linearVelocity = cmd_msg.linear.x;
  angularVelocity = cmd_msg.angular.z;
  // linear and angular velocities are converted to leftwheel and rightwheel velocities
  float vL = linearVelocity - (angularVelocity * WHEELS_Y_DISTANCE) / 2.0;
  float vR = linearVelocity + (angularVelocity * WHEELS_Y_DISTANCE) / 2.0;

  EncoderMeasurement left_encoder_meas = left_ctrl.Update(vL);
  float left_vel_mps = left_encoder_meas.delta_pos_m / left_encoder_meas.delta_time_sec;

  EncoderMeasurement right_encoder_meas = right_ctrl.Update(vR);
  float right_vel_mps = right_encoder_meas.delta_pos_m / right_encoder_meas.delta_time_sec;

  // Mean time between encoder measurements.
  float vel_dt = (left_encoder_meas.delta_time_sec + right_encoder_meas.delta_time_sec) / 2.0;

  // odometry
  float meas_linear_vel_mps = (left_vel_mps + right_vel_mps) / 2.0;
  float meas_angular_vel_rps = (right_vel_mps - left_vel_mps) / WHEELS_Y_DISTANCE;
  odometry.update(
      vel_dt,
      meas_linear_vel_mps,
      0,
      meas_angular_vel_rps);
  publishData();
}

// subscription callback function
void SubscriptionCallback(const void *msgin)
{
  Serial.println("[SUB] Got vel command");
}

bool CreateEntities()
{
  allocator = rcl_get_default_allocator();

  // Initialize options and set domain ID
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK)
  {
    return false;
  }

  // Initialize support with domain ID options
  if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK)
  {
    return false;
  }

  // Clean up initialization options
  if (rcl_init_options_fini(&init_options) != RCL_RET_OK)
  {
    return false;
  }

  // Initialize node and rest of entities
  if (rclc_node_init_default(&node, ROS_NODE_NAME, "", &support) != RCL_RET_OK)
  {
    return false;
  }

  // Create publisher
  if (rclc_publisher_init_default(&publisher, &node,
                                  ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                                  "odom/unfiltered") != RCL_RET_OK)
  {
    return false;
  }

  // Create subscriber
  if (rclc_subscription_init_default(&subscriber, &node,
                                     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                     "cmd_vel") != RCL_RET_OK)
  {
    return false;
  }

  // timer function for controlling the motor base. At every samplingT time
  // MotorControllerCallback function is called
  // Here I had set SamplingT=10 Which means at every 10 milliseconds MotorControllerCallback function is called
  if (rclc_timer_init_default(
          &control_timer,
          &support,
          RCL_MS_TO_NS(CONTROL_TIMER_PERIOD_MS),
          MotorControllerCallback) != RCL_RET_OK)
  {
    return false;
  }

  // Initialize executor
  // Handle for subscription and control timer
  if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK)
  {
    return false;
  }

  // Add subscriber to executor
  if (rclc_executor_add_subscription(&executor, &subscriber, &cmd_msg,
                                     &SubscriptionCallback, ON_NEW_DATA) != RCL_RET_OK)
  {
    return false;
  }

  if (rclc_executor_add_timer(&executor, &control_timer) != RCL_RET_OK)
  {
    return false;
  }

  return true;
}

void DestroyEntities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_ret_t rc = RCL_RET_OK;
  rc = rcl_timer_fini(&control_timer);
  rc = rcl_subscription_fini(&subscriber, &node);
  rc = rcl_publisher_fini(&publisher, &node);
  rclc_executor_fini(&executor);
  rc = rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void HandleConnectionState()
{
  switch (connection_state)
  {
  case ConnectionState::kWaitingForAgent:
    if (RMW_RET_OK == rmw_uros_ping_agent(200, 3))
    {
      Serial.println("[ROS] Agent found, establishing connection...");
      connection_state = ConnectionState::kConnecting;
    }
    break;

  case ConnectionState::kConnecting:
    if (CreateEntities())
    {
      Serial.println("[ROS] Connected and ready!");
      connection_state = ConnectionState::kConnected;
    }
    else
    {
      Serial.println("[ROS] Connection failed, retrying...");
      connection_state = ConnectionState::kWaitingForAgent;
    }
    break;

  case ConnectionState::kConnected:
    if (RMW_RET_OK != rmw_uros_ping_agent(200, 3))
    {
      Serial.println("[ROS] Agent disconnected!");
      connection_state = ConnectionState::kDisconnected;
    }
    else
    {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(ROS_EXECUTOR_TIMEOUT));
    }
    break;

  case ConnectionState::kDisconnected:
    DestroyEntities();
    Serial.println("[ROS] Waiting for agent...");
    connection_state = ConnectionState::kWaitingForAgent;
    break;

  default:
    break;
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("[INIT] Starting micro-ROS node...");

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  left_ctrl.Setup();
  right_ctrl.Setup();

  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);

  delay(2000);
  connection_state = ConnectionState::kWaitingForAgent;
}

void loop()
{
  HandleConnectionState();
  delay(5); // Prevent tight loop, but don't interfere with update rates.
}
