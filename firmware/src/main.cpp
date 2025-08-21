#include <Arduino.h>
#include <ArduinoOTA.h>

#include <MQTT.h> // https://github.com/256dpi/arduino-mqtt

#include "motor_ctrl.h"
#include "encoder_ctrl.h"

#include "WifiManagerParamHelper.h"

using namespace wheel_hal;

////// Pin declarations
static constexpr int8_t PIN_LED = 2;

//// Left wheel
// Control pins need to be able to generate PWM
static constexpr int8_t PIN_R_FORW = 25;
static constexpr int8_t PIN_R_BACK = 26;
// Encoder needs to support interrupts.
static constexpr int8_t PIN_R_ENCODER = 14;

//// Right wheel
// Control pins need to be able to generate PWM
static constexpr int8_t PIN_L_FORW = 32;
static constexpr int8_t PIN_L_BACK = 33;
// Encoder needs to support interrupts.
static constexpr int8_t PIN_L_ENCODER = 27;

////// Parameters of the robot
// Units in meters
static constexpr float WHEELS_Y_DISTANCE = .173;
static constexpr float WHEEL_RADIUS = 0.0245;
static constexpr float WHEEL_CIRCUMFERENCE = 0.153;
//// Encoder value per revolution of left wheel and right wheel
static constexpr unsigned TICK_PER_REVOLUTION = 4000;
// Minimum PWM value for movement
static constexpr unsigned PWM_THRESHOLD = 100;

////// Control parameters
static constexpr float MOVE_SPEED_PERCENT = 10.0;
static constexpr float ROTATE_SPEED_PERCENT = 10.0;

///// Processing Parameters
// Run main loop at 50Hz
static constexpr unsigned LOOP_SLEEP_DURATION_MILLIS = 20;

////// Network Parameters
static constexpr const char *AP_NAME = "MintBotAP";
static constexpr const char *MQTT_CMD_TOPIC = "/mint_bot/cmd";
static constexpr const char *MQTT_STATE_TOPIC = "/mint_bot/state";

// Entries for the "Setup" page on the web portal.
WiFiManager wm;
WifiManagerParamHelper wm_helper(wm);
constexpr const char *SETTING_DEVICE_NAME = "device_name";
constexpr const char *SETTING_MQTT_SERVER = "mqtt_server";
constexpr const char *SETTING_MQTT_PORT = "mqtt_port";
constexpr const char *SETTING_MQTT_USERNAME = "mqtt_username";
constexpr const char *SETTING_MQTT_PASSWORD = "mqtt_password";

std::array<ParamEntry, 5> PARAMS = {
    ParamEntry(SETTING_DEVICE_NAME, "mint_bot"),
    ParamEntry(SETTING_MQTT_SERVER, "192.168.1.110"),
    ParamEntry(SETTING_MQTT_PORT, "1883"),
    ParamEntry(SETTING_MQTT_USERNAME, "public"),
    ParamEntry(SETTING_MQTT_PASSWORD, "public")};

////// Global variables

// Motor interface
static SinglePinEncoderCtrl left_encoder(WHEEL_RADIUS, TICK_PER_REVOLUTION, PIN_L_ENCODER, INPUT, FALLING);
static SinglePinEncoderCtrl right_encoder(WHEEL_RADIUS, TICK_PER_REVOLUTION, PIN_R_ENCODER, INPUT, FALLING);
static AT8236MotorCtrl left_motor(PIN_L_FORW, PIN_L_BACK, PWM_THRESHOLD);
static AT8236MotorCtrl right_motor(PIN_R_FORW, PIN_R_BACK, PWM_THRESHOLD);

// Network interface
static MQTTClient mqtt_client;
static WiFiClient wifi_client;
static long long next_reconnect = 0;

///// Motor control classes

enum class CommandType
{
  STOP = 0,
  MOVE = 1,
  ROTATE = 2
};

class MotionController
{
public:
  void Setup()
  {
    left_encoder.SetupPins();
    left_motor.SetupPins();
    right_encoder.SetupPins();
    right_motor.SetupPins();
    Stop();
  }

  void StartMovement(CommandType type, float distance)
  {
    switch (type)
    {
    case CommandType::STOP:
    {
      Stop();
    }
    break;
    case CommandType::MOVE:
    {
      ClearEncoders();
      bool is_reverse = distance < 0;
      left_motor.SetSpeed(MOVE_SPEED_PERCENT, is_reverse);
      right_motor.SetSpeed(MOVE_SPEED_PERCENT, is_reverse);
      // The distance in meters each wheel should turn.
      // Stop point will use the average between the two wheels.
      target_distance_ = abs(distance);
    }
    break;
    case CommandType::ROTATE:
    {
      ClearEncoders();
      bool is_reverse = distance < 0;
      left_motor.SetSpeed(ROTATE_SPEED_PERCENT, is_reverse);
      right_motor.SetSpeed(ROTATE_SPEED_PERCENT, !is_reverse);
      // This is the distance each wheel should turn to achieve the desired rotation.
      // Here distance is expected in radians.
      // Stop point will use the average between the two wheels.
      target_distance_ = abs(distance) / (WHEELS_Y_DISTANCE * M_PI);
    }
    break;
    }
    Serial.printf("Target Distance: %fm\n", target_distance_);
  }

  void Update()
  {
    if (target_distance_ == 0)
    {
      return;
    }

    BaseEncoderCtrl *encoders[2] = {&left_encoder, &right_encoder};
    float average_distance = 0;

    for (int i = 0; i < 2; i++)
    {
      auto encoder = encoders[i];
      EncoderMeasurement encoder_meas = encoder->GetEncoderMeasurement();
      wheel_distance_[i] += encoder_meas.delta_pos_m;
      average_distance += wheel_distance_[i] / 2.0;
    }

    // Check if average distance exceeds target
    if (average_distance > target_distance_)
    {
      Serial.printf("Target Reached %f/%f - LW:%f RW:%f\n", average_distance, target_distance_, wheel_distance_[0], wheel_distance_[1]);
      Stop();
    }
  }

  bool IsMoving() const { return target_distance_ != 0; }

private:
  float target_distance_ = 0;
  float wheel_distance_[2] = {0};

  void ClearEncoders()
  {
    left_encoder.GetEncoderMeasurement();
    right_encoder.GetEncoderMeasurement();
    memset(wheel_distance_, 0, sizeof(wheel_distance_));
  }

  void Stop()
  {
    left_motor.SetSpeed(0);
    right_motor.SetSpeed(0);
    target_distance_ = 0;
  }
};

static MotionController motion_ctrl;
// Keeps track of the last published state to check when it changed
static bool moving_state = true;

static CommandType new_cmd_type = CommandType::STOP;
static float new_cmd_distance = 0;
static std::atomic<bool> new_cmd_ready{false};

bool ReconnectMQTT()
{
  const char *device_name = wm_helper.GetSettingValue(SETTING_DEVICE_NAME);
  const char *mqtt_server = wm_helper.GetSettingValue(SETTING_MQTT_SERVER);
  int mqtt_port = 0;
  WifiManagerParamHelper::str2int(wm_helper.GetSettingValue(SETTING_MQTT_PORT), &mqtt_port);
  const char *mqtt_username = wm_helper.GetSettingValue(SETTING_MQTT_USERNAME);
  const char *mqtt_password = wm_helper.GetSettingValue(SETTING_MQTT_PASSWORD);

  if (next_reconnect > millis() || !WiFi.isConnected() ||
      strlen(device_name) == 0 ||
      strlen(mqtt_server) == 0)
  {
    return false;
  }

  if (mqtt_port == 0)
  {
    Serial.printf("Invalid port specified \"%s\".\n", wm_helper.GetSettingValue(SETTING_MQTT_PORT));
  }

  mqtt_client.setHost(mqtt_server, mqtt_port);
  Serial.print("Attempting MQTT connection...");
  // Attempt to connect
  if (mqtt_client.connect(device_name, mqtt_username, mqtt_password))
  {
    Serial.println("connected");
    mqtt_client.subscribe(MQTT_CMD_TOPIC);
    return true;
  }
  else
  {
    Serial.print("failed, rc=");
    Serial.print(mqtt_client.returnCode());
    Serial.println(" try again in 5 seconds");
    // Wait 5 seconds before retrying
    next_reconnect = millis() + 5000;
  }

  return false;
}

float GetDistanceFromMessage(String &payload)
{
  int idx = payload.indexOf(",");
  if (idx > 0 && idx < payload.length() - 1)
  {
    // atof returns 0 on invalid values.
    return atof(payload.substring(idx + 1).c_str());
  }
  return 0;
}

void MessageReceived(String &topic, String &payload)
{
  Serial.println("incoming: " + topic + " - " + payload);

  if (topic == MQTT_CMD_TOPIC)
  {
    if (payload.startsWith("STOP"))
    {
      new_cmd_type = CommandType::STOP;
      new_cmd_distance = 0;
      new_cmd_ready = true;
    }
    else if (payload.startsWith("MOVE"))
    {
      new_cmd_type = CommandType::MOVE;
      new_cmd_distance = GetDistanceFromMessage(payload);
      new_cmd_ready = true;
    }
    else if (payload.startsWith("ROTATE"))
    {
      new_cmd_type = CommandType::ROTATE;
      new_cmd_distance = GetDistanceFromMessage(payload);
      new_cmd_ready = true;
    }
  }
}

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  motion_ctrl.Setup();

  ArduinoOTA
      .onStart([]()
               {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type); })
      .onEnd([]()
             { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total)
                  { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
      .onError([](ota_error_t error)
               {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed"); });
  ArduinoOTA.begin();

  wm_helper.Init(0xBEEF, PARAMS.data(), PARAMS.size());
  // Configure the WiFiManager
  wm.setConfigPortalBlocking(false);
  wm.setParamsPage(true);
  wm.setTitle("Pixels Dice Gateway");

  // Automatically connect using saved credentials if they exist
  // If connection fails it starts an access point with the specified name
  if (wm.autoConnect(AP_NAME))
  {
    Serial.println("Auto connect succeeded!");
  }
  else
  {
    Serial.println("Config portal running");
  }

  mqtt_client.onMessage(MessageReceived);
  mqtt_client.begin(wifi_client);
}

void loop()
{
  ///// Manage Motor Control and Odometry
  if (new_cmd_ready)
  {
    motion_ctrl.StartMovement(new_cmd_type, new_cmd_distance);
    new_cmd_ready = false;
  }
  motion_ctrl.Update();

  ///// WifiManager Processing
  // Manage configuration web portal.
  // Normally, it shuts down once the device is connected, but we want it to always be running.
  if (!wm.getWebPortalActive() && WiFi.isConnected())
  {
    Serial.println("Forcing web portal to start.");
    wm.startWebPortal();
  }
  wm.process();

  ///// Manage MQTT
  if (!mqtt_client.connected())
  {
    ReconnectMQTT();
  }
  else
  {
    if (moving_state != motion_ctrl.IsMoving())
    {
      const char *msg = (motion_ctrl.IsMoving()) ? "1" : "0";
      mqtt_client.publish(MQTT_STATE_TOPIC, msg, 1, true, 0);
      moving_state = motion_ctrl.IsMoving();
    }
  }
  mqtt_client.loop();

  ArduinoOTA.handle();
  delay(LOOP_SLEEP_DURATION_MILLIS);
}
