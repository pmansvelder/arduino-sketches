#include <Arduino.h>
#include <WiFi.h>
#include <CircuitOS.h>
#include <Spencer.h>
#include <PreparedStatement.h>
#include "PubSubClient.h"           //PubSubClient.h Library from Knolleary, must be adapted: #define MQTT_MAX_PACKET_SIZE 512
#include "ArduinoJson.h"
#define BUFFERSIZE 512              // default 100, should be 512
#define MQTT_MAX_PACKET_SIZE 512    // max size of mqtt payload

//PubSubClient mqttClient;

#include "secrets.h"
#define ESP32_clientID "Spencer"

WiFiClient espClient;
PubSubClient mqttClient(espClient); //lib required for mqtt

StaticJsonDocument<512> doc;        // default 512
char messageBuffer[BUFFERSIZE];
String ip = "";
bool startsend = HIGH;

#define DEBUG 1 // Zet debug mode aan

// Vul hier de naam in waarmee de Arduino zich aanmeldt bij MQTT, tevens het unique_id bij Home Assistant
#define CLIENT_ID  "spencer"
// Vul hier de naam in waarmee de Arduino zich aanmeldt bij Home Assistant
#define DISCOVERY_ID  "Spencer"
#define MODEL_ID  "Spencer"
#define MANUFACTURER_ID  "CircuitMess"
const String hostname = CLIENT_ID;
// base for Home Assistant MQTT discovery (must be configured in configuration.yaml)
String config_topic_base = "homeassistant";
// prefix for inidvidual items
String item_prefix = "spencer";

// MQTT Discovery relays
// Vul hier het aantal gebruikte relais in en de pinnen waaraan ze verbonden zijn
const byte NumberOfRelays = 1;
const byte RelayPins[] = {0};
const bool RelayInitialState[] = {LOW};
const char* const SwitchNames[] = {"Deurbel"};
char* state_topic_relays = "domus/spencer/stat/relay";

// MQTT Discovery lights
// Vul hier het aantal gebruikte lampen in en de pinnen waaraan ze verbonden zijn
const byte NumberOfLights = 1;
const byte LightPins[] = {0};
const bool LightInitialState[] = {LOW};
const bool LightBrightness[] = {true};
byte LightValue[] = {0};
const char* const LightNames[] = {"Display"};
const char* state_topic_lights = "domus/spencer/stat/light";
const char* cmd_topic_lights = "domus/spencer/cmd/light";

// MQTT Discovery covers
const byte NumberOfCovers = 0;
const byte CoverDir[] = {}; // relay numbers for direction
const byte CoverPulse[] = {}; // relay numbers for motor pulses
byte CoverState[] = {}; // 0 = open, 1 = opening, 2 = closed, 3 = closing, 4 = stopped
int CoverPos[] = {}; // position 100 = open
int CoverStart[] = {}; // start position
int CoverSetPos[] = {}; // set position (255 = not set)
String CoverClasses[] = {}; // https://www.home-assistant.io/integrations/cover/
const char* const CoverNames[] = {};
const long CoverDelay[] = {}; // time to wait for full open or close
const char* state_topic_covers = "domus/spencer/uit/screen"; // Screens (zonwering)

// MQTT Discovery locks
const byte NumberOfLocks = 0;
const byte LockPulse[] = {}; // relay numbers for lock pulses (index on PulseRelayPins)
byte LockState[] = {};  // status of locks: 0 = unlocked, 1 = locked
const char* const LockNames[] = {"Haldeurslot", "VoordeurSlot"};
const long LockDelay[] = {}; // pulse time for locks
const char* state_topic_locks = "domus/spencer/stat/lock"; // Locks (sloten)

// MQTT Discovery pirs (binary_sensors)
const byte NumberOfPirs = 0;
const byte PirSensors[] = {};
const int PirDebounce[] = {}; // debounce time for pir or door sensor
long PirLastActivityTimes[] = {};
static byte lastPirStates[] = {};
const bool PirInitialState[] = {};
int PreviousDetects[] = {}; // Statusvariabele PIR sensor
byte PirState[] = {};
const char* const PirNames[] = {};
const char* const PirClasses[] = {};
const char* state_topic_pirs = "domus/spencer/uit/pir";

// MQTT Discovery buttons (device triggers)
const int NumberOfButtons = 1;
const int ButtonPins[] = {11};
static byte lastButtonStates[] = {0};
long lastActivityTimes[] = {0};
long LongPressActive[] = {0};
const char* state_topic_buttons = "domus/spencer/uit/button";

// MQTT Discovery sensors (sensors)
const int NumberOfSensors = 1;
const char* const SensorNames[] = {"Runtime"};
const char* const SensorTypes[] = {"TIME"};
const char* const SensorClasses[] = {""};
const char* const SensorUnits[] = {"s"};
const char* state_topic_sensors = "domus/spencer/uit/sensor";

PreparedStatement* statement = nullptr;
bool synthesizing = false;

float volume = 4.0;

// MQTT topics
const char* topic_out = "domus/spencer/uit";
const char* topic_in = "domus/spencer/in";

long NextRandomTime = 0, TimeNow;
const int RandomInterval = 15000;

#if defined (DEBUG)
bool debug = true;
#else
bool debug = false;
#endif
void ShowDebug(String tekst) {
  if (debug) {
    Serial.println(tekst);
  }
}

void RandomAnimation() {
  int RandomNumber;
  String randomfile;
  RandomNumber = random(1, 11    ); // number from 1 to 10
  randomfile = "GIF-idle" + String(RandomNumber) + ".gif";
  Serial.println("Idle pattern : " + randomfile);
  char char_array[randomfile.length() + 1];
  randomfile.toCharArray(char_array, randomfile.length() + 1);
  LEDmatrix.startAnimation(new Animation(new SerialFlashFileAdapter(char_array)), true);
}

void speechPlay(TTSError error, CompositeAudioFileSource* source) {
  synthesizing = false;
  if (error != TTSError::OK) {
    Serial.printf("Text to speech error %d: %s\n", error, TTSStrings[(int) error]);
    delete source;
    delete statement;
    statement = nullptr;
    return;
  }
  Playback.playMP3(source);
  Playback.setPlaybackDoneCallback([]() {

  });
  delete statement;
  statement = nullptr;

}

void SpeakOut(String SomeWords) {
  if (synthesizing) {
    Serial.println("Another speech synthesis operation is already pending");
  } else {
    synthesizing = true;
    delete statement;
    statement = new PreparedStatement();
    statement->addTTS(SomeWords);
    statement->prepare(speechPlay);
  }
}

void BTN_press() {
  Playback.playMP3(SampleStore::load(SampleGroup::Special, "startup"));
  LEDmatrix.startAnimation(new Animation(new SerialFlashFileAdapter("GIF-listen.gif")), true);
  ShowDebug( "Button0 pressed" );
  String messageString = "Button0";
  messageString.toCharArray(messageBuffer, messageString.length() + 1);
  mqttClient.publish(state_topic_buttons, messageBuffer);
}

void reconnect() {
  LEDmatrix.startAnimation(new Animation(new SerialFlashFileAdapter("GIF-wifi.gif")), true);
  while (!mqttClient.connected()) {
    WiFi.begin(SECRET_SSID, SECRET_PASS);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    mqttClient.setServer(MQTTSERVER, 1883);
    ip = String (WiFi.localIP()[0]);
    ip = ip + ".";
    ip = ip + String (WiFi.localIP()[1]);
    ip = ip + ".";
    ip = ip + String (WiFi.localIP()[2]);
    ip = ip + ".";
    ip = ip + String (WiFi.localIP()[3]);

    Serial.println("Attempting MQTT connection...");
    LEDmatrix.startAnimation(new Animation(new SerialFlashFileAdapter("GIF-noWifi.gif")), true);
    if (mqttClient.connect(ESP32_clientID)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      mqttClient.publish(topic_out, "Spencer connected to MQTT");
      // ... and resubscribe
      mqttClient.subscribe(topic_in);

    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void connectmqtt()
{
  LEDmatrix.startAnimation(new Animation(new SerialFlashFileAdapter("GIF-wifi.gif")), true);
  mqttClient.connect(ESP32_clientID);  // ESP will connect to mqtt broker with clientID
  {
    Serial.println("Spencer connected to MQTT");
    // Once connected, publish an announcement...

    // ... and resubscribe
    mqttClient.subscribe(topic_in); //topic=Demo
    mqttClient.publish(topic_out,  "Spencer connected to MQTT");

    if (!mqttClient.connected())
    {
      reconnect();
    }
  }
}

void setDeviceInfo(const char* configtopic) {
  JsonObject device = doc.createNestedObject("dev");
  JsonArray identifiers = device.createNestedArray("ids");
  identifiers.add(CLIENT_ID);
  JsonArray connections = device.createNestedArray("cns");
  connections.add(serialized("[\"ip\",\"" + String(ip) + "\"]"));
#if !defined(UNO_WIFI)
  //  connections.add(serialized("[\"mac\",\"" + mac2String(mac) + "\"]"));
#endif
  device["name"] = DISCOVERY_ID;
  device["mdl"] = MODEL_ID;
  device["mf"] = MANUFACTURER_ID;
  size_t n = serializeJson(doc, messageBuffer);
  ShowDebug("Sending MQTT config on topic:");
  ShowDebug(configtopic);
  ShowDebug("Config json:");
  ShowDebug(messageBuffer);
  if (mqttClient.publish(configtopic, messageBuffer, n)) {
    ShowDebug("...MQTT config sent.");
  }
  else {
    ShowDebug("...publish failed, either connection lost, or message too large.");
  }
}

void reportMQTTdisco() {
  // discovery data for relays
  for (int i = 0; i < NumberOfRelays ; i++) {
    doc.clear();
    doc["name"] = SwitchNames[i];
    doc["uniq_id"] = item_prefix + "_switch" + String(i + 1);
    doc["stat_t"] = state_topic_relays;
    doc["cmd_t"] = topic_in;
    if (!RelayInitialState[i]) {
      doc["pl_on"] = "R" + String(i) + "1";
      doc["pl_off"] = "R" + String(i) + "0";
    }
    else {
      doc["pl_on"] = "R" + String(i) + "0";
      doc["pl_off"] = "R" + String(i) + "1";
    }
    doc["stat_on"] = "on";
    doc["stat_off"] = "off";
    doc["val_tpl"] = "{{value_json.POWER" + String(i) + "}}";
    setDeviceInfo((config_topic_base + "/switch/" + item_prefix + "_switch" + String(i + 1) + "/config").c_str());
  }
  // discovery data for lights
  for (int i = 0; i < NumberOfLights ; i++) {
    doc.clear();
    doc["name"] = LightNames[i];
    doc["uniq_id"] = item_prefix + "_light" + String(i + 1);
    doc["stat_t"] = state_topic_lights + String(i + 1);
    doc["cmd_t"] = cmd_topic_lights + String(i + 1);
    doc["schema"] = "json";
    doc["brightness"] = LightBrightness[i];
    mqttClient.subscribe((cmd_topic_lights + String(i + 1)).c_str());
    setDeviceInfo((config_topic_base + "/light/" + item_prefix + "_light" + String(i + 1) + "/config").c_str());
  }
  // discovery data for covers
  for (int i = 0; i < NumberOfCovers ; i++ ) {
    doc.clear();
    doc["name"] = CoverNames[i];
    doc["uniq_id"] = item_prefix + "_cover" + String(i + 1);
    doc["dev_cla"] = CoverClasses[i];
    doc["pos_t"] = state_topic_covers;
    doc["cmd_t"] = topic_in;
    doc["pl_open"] = "O" + String(i + 1);
    doc["pl_cls"] = "C" + String(i + 1);
    doc["pl_stop"] = "S" + String(i + 1);
    doc["pos_open"] = 100;
    doc["pos_clsd"] = 0;
    doc["set_pos_t"] = topic_in;
    doc["set_pos_tpl"] = "{ \"POSITION" + String(i + 1) +  "\": {{ position }} }";
    doc["pos_tpl"] = "{{value_json.POSITION" + String(i + 1) + "}}";
    setDeviceInfo((config_topic_base + "/cover/" + item_prefix + "_cover" + String(i + 1) + "/config").c_str());
  }
  // discovery data for locks
  for (int i = 0; i < NumberOfLocks ; i++ ) {
    doc.clear();
    doc["name"] = LockNames[i];
    doc["uniq_id"] = item_prefix + "_lock" + String(i + 1);
    doc["stat_t"] = state_topic_locks;
    doc["cmd_t"] = topic_in;
    doc["pl_lock"] = "L" + String(i + 1);
    doc["pl_unlk"] = "U" + String(i + 1);
    doc["state_locked"] = "LOCKED";
    doc["state_unlocked"] = "UNLOCKED";
    doc["val_tpl"] = "{{value_json.LOCK" + String(i + 1) + "}}";
    setDeviceInfo((config_topic_base + "/lock/" + item_prefix + "_lock" + String(i + 1) + "/config").c_str());
  }
  // discover data for pirs (binary_sensors)
  for (int i = 0; i < NumberOfPirs ; i++ ) {
    doc.clear();
    doc["name"] = PirNames[i];
    doc["uniq_id"] = item_prefix + "_pir" + String(i + 1);
    doc["stat_t"] = state_topic_pirs;
    doc["device_class"] = PirClasses[i];
    doc["pl_on"] = "ON";
    doc["pl_off"] = "OFF";
    doc["val_tpl"] = " {{value_json.PIR" + String(i + 1) + "}}";
    setDeviceInfo((config_topic_base + "/binary_sensor/" + item_prefix + "_pir" + String(i + 1) + "/config").c_str());
  }
  // discover data for buttons (triggers)
  for (int i = 0; i < NumberOfButtons ; i++ ) {
    doc.clear();
    doc["automation_type"] = "trigger";
    doc["topic"] = state_topic_buttons;
    doc["type"] = "button_short_press";
    doc["subtype"] = "button_" + String(i + 1);
    doc["payload"] = "Button" + String(i);
    setDeviceInfo((config_topic_base + "/device_automation/" + item_prefix + "_button" + String(i + 1) + "/config").c_str());
    doc.clear();
    doc["automation_type"] = "trigger";
    doc["topic"] = state_topic_buttons;
    doc["type"] = "button_long_press";
    doc["subtype"] = "button_" + String(i + 1);
    doc["payload"] = "Button" + String(i) + "_long";
    setDeviceInfo((config_topic_base + "/device_automation/" + item_prefix + "_button" + String(i + 1) + "_long" + "/config").c_str());
  }
  // end send config data for MQTT discovery
  // discover data for sensors (sensors)
  for (int i = 0; i < NumberOfSensors ; i++ ) {
    doc.clear();
    doc["name"] = SensorNames[i];
    doc["uniq_id"] = item_prefix + "_sensor" + String(i + 1);
    doc["stat_t"] = state_topic_sensors;
    if (SensorClasses[i] != "") {
      doc["device_class"] = SensorClasses[i];
    }
    if (SensorUnits[i] != "") {
      doc["unit_of_meas"] = SensorUnits[i];
    }
    doc["val_tpl"] = " {{value_json.sensor" + String(i + 1) + " | round (1) }}";
    setDeviceInfo((config_topic_base + "/sensor/" + item_prefix + "_sensor"  + String(i + 1) + "/config").c_str());
  }
  //  end send config data for MQTT discovery
}

void report_state_light(int index) {
  StaticJsonDocument<64> outputdoc;
  outputdoc.clear();
  if (LightBrightness[index]) {
    if (LightValue[index] > 0) {
      outputdoc["state"] = "ON";
    }
    else {
      outputdoc["state"] = "OFF";
    }
    outputdoc["brightness"] = LEDmatrix.getBrightness();
  }
  else if (LightPins[index] < 100)
  {
    if (digitalRead(LightPins[index]) == LightInitialState[index])
    {
      outputdoc["state"] = "OFF";
    }
    else {
      outputdoc["state"] = "ON";
    }
  }
#if defined(MCP_present)
  else {
    if (mcp.digitalRead(LightPins[index] - 100) == LightInitialState[index])
    {
      outputdoc["state"] = "OFF";
    }
    else {
      outputdoc["state"] = "ON";
    }
  }
#endif
  serializeJson(outputdoc, messageBuffer);
  ShowDebug("Sending MQTT state for lights...");
  ShowDebug(messageBuffer);
  mqttClient.publish((state_topic_lights + String(index + 1)).c_str(), messageBuffer);
}

void SetLightState(int light, String state) {
  if (state == "ON") {
    if (LightPins[light] < 100) {
      ShowDebug("Set light " + String(LightPins[light]) + " on.");
      digitalWrite(LightPins[light], !LightInitialState[light]);
    }
  }
  else if (state == "OFF") {
    if (LightPins[light] < 100) {
      ShowDebug("Set light " + String(LightPins[light]) + " off.");
      LEDmatrix.setBrightness(0);
      LightValue[light] = 0;
    }
  }
  else {
    if (LightPins[light] < 100) {
      ShowDebug("Setting pwm value " + String(LightValue[light]) + "on pin " + String(LightPins[light]));
      if (LightInitialState[light]) {
        //          analogWrite(LightPins[light], 255 - LightValue[light]);
        LEDmatrix.setBrightness(255 - LightValue[light]);
      }
      else {
        LEDmatrix.setBrightness(LightValue[light]);
        //          analogWrite(LightPins[light], LightValue[light]);
      }
    }
  }
  report_state_light(light);
}

void callback(char* topic, byte * payload, byte length) {
  char msgBuffer[BUFFERSIZE];
  payload[length] = '\0'; // terminate string with 0
  String strPayload = String((char*)payload);  // convert to string
  ShowDebug("Message arrived");
  ShowDebug(topic);
  ShowDebug(strPayload);

  byte RelayPort;
  byte RelayValue;

  if (String(topic).indexOf(cmd_topic_lights) >= 0) {
    doc.clear();
    deserializeJson(doc, strPayload);
    int index = (topic[strlen(topic) - 1]) - '0';
    if (LightBrightness[index - 1] && doc.containsKey("brightness")) {
      LightValue[index - 1] = doc["brightness"];
      SetLightState(index - 1, doc["brightness"]);
    }
    else {
      if (LightBrightness[index - 1]) {
        LightValue[index - 1] = 255;
      }
      SetLightState(index - 1, doc["state"]);
    }
  }
  else if (strPayload == "R01")  {
    // 'Show IP' commando
    Playback.playMP3(SampleStore::load(SampleGroup::Special, "badum1"));
    LEDmatrix.startAnimation(new Animation(new SerialFlashFileAdapter("GIF-sun.gif")), true);
    mqttClient.publish(topic_out, "LED turned ON");
    SpeakOut("There is somebody at the door");
  }
  else if (strPayload[0] == 'V')  {
    // 'Volume' commando
    ShowDebug(String(strPayload[1] - '0'));
    float Volume = (float)(strPayload[1] - '0') / 10.0;
    ShowDebug("Setting volume to " + String(Volume));
    Playback.setVolume(Volume);
    Playback.playMP3(SampleStore::load(SampleGroup::Special, "startup"));
  }

  else if (strPayload == "IP")  {
    // 'Show IP' commando
    mqttClient.publish(topic_out, ip.c_str());// publish IP nr
  }
  else {
    // Onbekend commando
    ShowDebug("Unknown value");
    mqttClient.publish(topic_out, "Unknown command");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  randomSeed(analogRead(0));
  Spencer.begin();
  Spencer.loadSettings();
  LEDmatrix.setBrightness(10);
  LEDmatrix.startAnimation(new Animation(new SerialFlashFileAdapter("GIF-noWifi.gif")), true);
  Input::getInstance()->setBtnPressCallback(BTN_PIN, BTN_press);
  Serial.println("Starting connecting WiFi.");
  delay(500);
  int MaxTries = 10;
  int CurrentTry = 0;
  WiFi.begin(SECRET_SSID, SECRET_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    CurrentTry += 1;
    Serial.print(".");
    if (CurrentTry > MaxTries) {
      esp_restart();
    }
  }
  ip = String (WiFi.localIP()[0]);
  ip = ip + ".";
  ip = ip + String (WiFi.localIP()[1]);
  ip = ip + ".";
  ip = ip + String (WiFi.localIP()[2]);
  ip = ip + ".";
  ip = ip + String (WiFi.localIP()[3]);
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  mqttClient.setServer(MQTTSERVER, 1883);
  mqttClient.setCallback(callback);
  connectmqtt();
  LEDmatrix.clear();
  LEDmatrix.startAnimation(new Animation(new SerialFlashFileAdapter("GIF-smile.gif")), true);
}

void loop() {
  TimeNow = millis();
  if (TimeNow > NextRandomTime) {
    NextRandomTime = TimeNow + RandomInterval;
    RandomAnimation();
  }
  LoopManager::loop();

  if (!mqttClient.connected())
  {
    LEDmatrix.startAnimation(new Animation(new SerialFlashFileAdapter("GIF-noWifi.gif")), true);
    //    reconnect();
    esp_restart();
  }

  // ... then send all relay stats and discovery info when we've just started up....
  if (startsend) {
    reportMQTTdisco();
    startsend = false;
  }

  mqttClient.loop();
}
