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
static constexpr float WHEEL_RADIUS = 0.025;
//// Encoder value per revolution of left wheel and right wheel
static constexpr unsigned TICK_PER_REVOLUTION = 1160;
// Minimum PWM value for movement
static constexpr unsigned PWM_THRESHOLD = 100;

////// Control parameters
static constexpr float MOVE_SPEED_PERCENT = 10.0;
static constexpr float ROTATE_SPEED_PERCENT = 10.0;
static constexpr unsigned STOP_DURATION_MILLIS = 500;


////// Network Parameters
static constexpr const char *AP_NAME = "MintBotAP";
static constexpr const char *MQTT_CMD_TOPIC = "mint_bot/cmd";
static constexpr const char *MQTT_STATE_TOPIC = "mint_bot/state";

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

enum class EncoderCheckType
{
  MAX = 0,
  MIN = 1,
  MEAN = 2,
  LEFT = 3,
  RIGHT = 4
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

  void StartMovement(const String &cmd_string)
  {
    std::vector<String> args;
    int start = 0;
    int end = cmd_string.indexOf(",");
    while (end != -1)
    {
      args.push_back(cmd_string.substring(start, end));
      start = end + 1;
      end = cmd_string.indexOf(",", start);
    }
    args.push_back(cmd_string.substring(start));

    // ex. "STOP"
    if (cmd_string.startsWith("STOP"))
    {
      Stop();
    }
    // ex. "MOVE,-1.2"
    else if (args.size() == 2 && args[0] == "MOVE")
    {
      float distance = atof(args[1].c_str());
      check_type_ = EncoderCheckType::MEAN;
      ClearEncoders();
      bool is_reverse = distance < 0;
      left_motor.SetSpeed(MOVE_SPEED_PERCENT, is_reverse);
      right_motor.SetSpeed(MOVE_SPEED_PERCENT, is_reverse);
      // The distance in meters each wheel should turn.
      // Stop point will use the average between the two wheels.
      target_distance_ = abs(distance);
    }
    // ex. "ROTATE,3.14"
    else if (args.size() == 2 && args[0] == "ROTATE")
    {
      float angle = atof(args[1].c_str());
      check_type_ = EncoderCheckType::MEAN;
      ClearEncoders();
      bool is_reverse = angle < 0;
      left_motor.SetSpeed(ROTATE_SPEED_PERCENT, is_reverse);
      right_motor.SetSpeed(ROTATE_SPEED_PERCENT, !is_reverse);
      // This is the distance each wheel should turn to achieve the desired rotation.
      // Here distance is expected in radians.
      // Stop point will use the average between the two wheels.
      target_distance_ = abs(angle) / (WHEELS_Y_DISTANCE * M_PI);
    }
    // MANUAL,<LEFT PERCENT SPEED>,<RIGHT PERCENT SPEED>,<DISTANCE>,<ENCODER CHECK>
    // ex. "MANUAL,3.14"
    else if (args.size() == 5 && args[0] == "MANUAL")
    {
      float left_speed = atof(args[1].c_str());
      bool left_is_reverse = left_speed < 0;
      float right_speed = atof(args[2].c_str());
      bool right_is_reverse = right_speed < 0;
      target_distance_ = atof(args[3].c_str());
      String check_type_str = args[4];
      if (check_type_str == "MIN")
      {
        check_type_ = EncoderCheckType::MIN;
      }
      else if (check_type_str == "MAX")
      {
        check_type_ = EncoderCheckType::MAX;
      }
      else if (check_type_str == "MEAN")
      {
        check_type_ = EncoderCheckType::MEAN;
      }
      else if (check_type_str == "LEFT")
      {
        check_type_ = EncoderCheckType::LEFT;
      }
      else if (check_type_str == "RIGHT")
      {
        check_type_ = EncoderCheckType::RIGHT;
      }
      else
      {
        check_type_ = EncoderCheckType::MIN; // Default fallback
      }

      ClearEncoders();
      left_motor.SetSpeed(left_speed, left_is_reverse);
      right_motor.SetSpeed(right_speed, right_is_reverse);
    }
    else
    {
      return;
    }

    Serial.printf("Target Distance: %fm\n", target_distance_);
  }

  void Update()
  {
    if (stop_start_time_ != 0) {
      if (millis() - stop_start_time_ > STOP_DURATION_MILLIS) {
        left_motor.SetSpeed(0);
        right_motor.SetSpeed(0);
        stop_start_time_ = 0;
      }
      return;
    }

    if (target_distance_ == 0)
    {
      return;
    }

    BaseEncoderCtrl *encoders[2] = {&left_encoder, &right_encoder};

    for (int i = 0; i < 2; i++)
    {
      auto encoder = encoders[i];
      EncoderMeasurement encoder_meas = encoder->GetEncoderMeasurement();
      wheel_distance_[i] += encoder_meas.delta_pos_m;
    }

    float check_distance = 0;
    const char *check_str = "";

    switch (check_type_)
    {
    case EncoderCheckType::MIN:
    {
      check_str = "MIN";
      check_distance = min(wheel_distance_[0], wheel_distance_[1]);
    }
    break;
    case EncoderCheckType::MAX:
    {
      check_str = "MAX";
      check_distance = max(wheel_distance_[0], wheel_distance_[1]);
    }
    break;
    case EncoderCheckType::MEAN:
    {
      check_str = "MEAN";
      check_distance = (wheel_distance_[0] + wheel_distance_[1]) / 2.0;
    }
    break;
    case EncoderCheckType::LEFT:
    {
      check_str = "LEFT";
      check_distance = wheel_distance_[0];
    }
    break;
    case EncoderCheckType::RIGHT:
    {
      check_str = "RIGHT";
      check_distance = wheel_distance_[1];
    }
    break;
    }

    // Check if average distance exceeds target
    if (check_distance >= target_distance_)
    {
      Serial.printf("Target Reached %s %f/%f - LW:%f RW:%f\n", check_str, check_distance, target_distance_, wheel_distance_[0], wheel_distance_[1]);
      Stop();
    }
  }

  bool IsMoving() const { return target_distance_ != 0; }

private:
  float target_distance_ = 0;
  EncoderCheckType check_type_ = EncoderCheckType::MIN;
  float wheel_distance_[2] = {0};
  unsigned stop_start_time_ = 0;

  void ClearEncoders()
  {
    left_encoder.GetEncoderMeasurement();
    right_encoder.GetEncoderMeasurement();
    memset(wheel_distance_, 0, sizeof(wheel_distance_));
  }

  void Stop()
  {
    left_motor.Brake();
    right_motor.Brake();
    target_distance_ = 0;
    stop_start_time_ = millis();
  }
};

static MotionController motion_ctrl;
// Keeps track of the last published state to check when it changed
static bool moving_state = true;

static String new_cmd;
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

void MessageReceived(String &topic, String &payload)
{
  Serial.println("incoming: " + topic + " - " + payload);

  if (topic == MQTT_CMD_TOPIC)
  {
    new_cmd = payload;
    new_cmd_ready = true;
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
    String cmd = new_cmd;
    new_cmd_ready = false;
    motion_ctrl.StartMovement(cmd);
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
}
