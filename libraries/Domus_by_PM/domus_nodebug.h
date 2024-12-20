// library file for domus sketches

#define DOMUS_LIBRARY_VERSION "2024.01.18-1" // library version

const String version = VERSION;

#if defined(COVERS)
#include <EEPROM.h>
#include "EEPROMAnything.h"

struct config_t
{
  bool saved;
  int cover_pos[2];
} configuration;
#endif


#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}

#if defined(UNO_WIFI)
    #include <WiFiNINA.h>
    #include <utility/wifi_drv.h>
    WiFiClient NetClient;
    int status = WL_IDLE_STATUS;    // the Wifi radio's status
#else
    #include <Ethernet.h>           // Ethernet.h library
    EthernetClient NetClient;
#endif
#include "PubSubClient.h"           //PubSubClient.h Library from Knolleary, must be adapted: #define MQTT_MAX_PACKET_SIZE 512
#include <ArduinoJson.h>            // Arduino JSON library to create/parse JSON messages, for arduinojson 7
// #include <ArduinoJson.hpp>
#define BUFFERSIZE 512              // default 100, should be 512
#define MQTT_MAX_PACKET_SIZE 512    // max size of mqtt payload
#define DEBOUNCE_DELAY 150          // debounce delay for buttons
#define LONGPRESS_TIME 450          // time for press to be detected as 'long'
// StaticJsonDocument<512> doc;        // default 512 (deprecated, arduinojson 6)
// JsonDocument doc;                   // for arduinojson 7

PubSubClient mqttClient;

// calculated topic for incoming messages
const String topic_in_string = "domus/" + item_prefix + "/in";
const char* topic_in = topic_in_string.c_str();

// calculated topic for outgoing status messages
const String status_topic_string = "domus/" + item_prefix + "/status";
const char* status_topic = status_topic_string.c_str();
// Last will message
const char* last_will = "error";
byte willQoS = 0;
boolean willRetain = false;

#if defined(MS_present)
byte ms_state = 1;                  // present state of MS sensor: 0=preheat, 1=measure
float ms_value = 0;
long ms_millis, ms_measure_millis;
const long ms_heat_interval = 50;
#endif

#if defined(P1_meter)
P1Reader reader(&Serial1, P1_REQUEST_PIN);
unsigned long last_p1_read;
MyData last_p1_data;
#endif

// BMP280 pressure and temperature sensor
#if defined(BMP_present)
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>        // Adafruit BMP280 library
Adafruit_BMP280 bmp;                // I2C: SDA=20, SCL=21
#endif

#if defined(MCP_present)
#include <Wire.h>
#include "Adafruit_MCP23017.h"      // define type of MCP expander
Adafruit_MCP23017 mcp;
#endif

char messageBuffer[BUFFERSIZE];
String ip = "";
bool startsend = HIGH;              // flag for sending at startup
                                    // Vul hier het interval in waarmee gegevens worden verstuurd op MQTT
#define PUBLISH_DELAY 10000         // that is 10 second interval
long lastPublishTime;
                                    // Vul hier het interval in waarmee alle statussen worden verstuurd op MQTT
#define REPORT_DELAY 60000          // that is 60 seconds interval
long lastReportTime;
#if defined (DEBUG)
bool debug = true;
#else
bool debug = false;
#endif

void(* resetFunc) (void) = 0;       //declare reset function @ address 0

void ShowDebug(String tekst) {
  if (debug) {
  }
}
String mac2String(byte ar[]) {
  String s;
  for (byte i = 0; i < 6; ++i)
  {
    char buf[3];
    sprintf(buf, "%02X", ar[i]);
    s += buf;
    if (i < 5) s += ':';
  }
  return s;
}
#if defined(UNO_WIFI)
void set_rgb_led(byte red, byte green, byte blue)
{
  WiFiDrv::analogWrite(25, red);  // for configurable brightness
  WiFiDrv::analogWrite(26, green);  // for configurable brightness
  WiFiDrv::analogWrite(27, blue);  // for configurable brightness
}
#endif

void report_state_relay() {
  bool relaytest;
  StaticJsonDocument<64> doc;
  if (NumberOfRelays > 0) {
    for (int i = 0; i < NumberOfRelays; i++) {
      if (RelayPins[i] < 100) {
        relaytest = (digitalRead(RelayPins[i]) == RelayInitialState[i]);
      }
#if defined(MCP_present)
      else {
        relaytest = (mcp.digitalRead(RelayPins[i] - 100) == RelayInitialState[i]);
      }
#endif
      if (relaytest) {
        doc["POWER" + String(i)] = "off";
      }
      else {
        doc["POWER" + String(i)] = "on";
      }
      serializeJson(doc, messageBuffer);
    }
    ShowDebug(F("Sending MQTT state for relays..."));
    ShowDebug(messageBuffer);
    mqttClient.publish(state_topic_relays, messageBuffer);
  }
}

void report_pulse_relay(int index, bool state)
{
    StaticJsonDocument<20> doc;
    if (state) {
      doc["PULSE" + String(index)] = "on";
    }
    else {
      doc["PULSE" + String(index)] = "off";
    }
    serializeJson(doc, messageBuffer);
    ShowDebug(F("Sending MQTT state for pulse relays..."));
    ShowDebug(messageBuffer);
    mqttClient.publish(state_topic_pulserelays, messageBuffer);
}

void report_state_pulserelay() {
  bool relaytest;
  if (NumberOfPulseRelays > 0) {
    for (int i = 0; i < NumberOfPulseRelays; i++) {
      if (PulseRelayPins[i] < 100) {
        relaytest = (digitalRead(PulseRelayPins[i]) == PulseRelayInitialStates[i]);
      }
#if defined(MCP_present)
      else {
        relaytest = (mcp.digitalRead(PulseRelayPins[i] - 100) == PulseRelayInitialStates[i]);
      }
#endif
      report_pulse_relay(i,!relaytest);
    }
  }
}

#if defined(LIGHTS)
void report_state_light(int index) {
  StaticJsonDocument<20> outputdoc;
//   JsonDocument outputdoc;
  outputdoc.clear();
  if (LightBrightness[index]) {
    if (LightValue[index] > 0) {
      outputdoc["state"] = "ON";
    }
    else {
      outputdoc["state"] = "OFF";
    }
    outputdoc["brightness"] = LightValue[index];
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
  ShowDebug(F("Sending MQTT state for lights..."));
  ShowDebug(messageBuffer);
  mqttClient.publish((state_topic_lights + String(index + 1)).c_str(), messageBuffer);
}
#endif

#if defined(COVERS)
void report_state_cover() {
  if (NumberOfCovers > 0) {
    StaticJsonDocument<128> outputdoc;
//     JsonDocument outputdoc;
    outputdoc.clear();
    for (byte i = 0; i < NumberOfCovers ; i++) {
      if (CoverState[i] == 0) {
        outputdoc["COVER" + String(i + 1)] = "open";
      }
      else if (CoverState[i] == 1) {
        outputdoc["COVER" + String(i + 1)] = "opening";
      }
      else if (CoverState[i] == 2) {
        outputdoc["COVER" + String(i + 1)] = "closed";
      }
      else if (CoverState[i] == 3) {
        outputdoc["COVER" + String(i + 1)] = "closing";
      }
      else if (CoverState[i] == 4) {
        outputdoc["COVER" + String(i + 1)] = "stopped";
      }
      outputdoc["POSITION" + String(i + 1)] = CoverPos[i];
    }
    serializeJson(outputdoc, messageBuffer);
    ShowDebug(F("Sending MQTT state for covers..."));
    ShowDebug(messageBuffer);
    mqttClient.publish(state_topic_covers, messageBuffer);
    outputdoc.clear();
  }
}
#endif

#if defined(LOCKS)
void report_state_lock() {
   StaticJsonDocument<64> doc;
  if (NumberOfLocks > 0) {
    for (byte i = 0; i < NumberOfLocks ; i++) {
      if (LockState[i] == 0) {
        doc["LOCK" + String(i + 1)] = "UNLOCKED";
      }
      else {
        doc["LOCK" + String(i + 1)] = "LOCKED";
      }
    }
    serializeJson(doc, messageBuffer);
    ShowDebug(F("Sending MQTT state for locks..."));
    ShowDebug(messageBuffer);
    mqttClient.publish(state_topic_locks, messageBuffer);
  }
}
#endif

void report_state_pir() {
  StaticJsonDocument<64> doc;
  if (NumberOfPirs > 0) {
    for (byte i = 0; i < NumberOfPirs ; i++) {
      if (PirState[i] == 0) {
        doc["PIR" + String(i + 1)] = "OFF";
      }
      else {
        doc["PIR" + String(i + 1)] = "ON";
      }
    }
    serializeJson(doc, messageBuffer);
    ShowDebug(F("Sending MQTT state for pirs..."));
    ShowDebug(messageBuffer);
    mqttClient.publish(state_topic_pirs, messageBuffer);
  }
}

void report_state() {
  // send general status of controller
  mqttClient.publish(status_topic, "ok");
  // send data for relays
  report_state_relay();
  report_state_pulserelay();
#if defined(LIGHTS)
  // send data for lights
  if (NumberOfLights > 0) {
    for (int index = 0; index < NumberOfLights ; index++) {
      report_state_light(index);
    }
  }
#endif
#if defined(COVERS)
  // send data for covers
  report_state_cover();
#endif  
#if defined(LOCKS)
  // send data for locks
  report_state_lock();
#endif
  // send data for pirs
  report_state_pir();
  // end send state data for MQTT discovery
}

void heartbeat() {
  ShowDebug(F("Heartbeat sent!"));
  pinMode(heartbeatPin, OUTPUT);
  digitalWrite(heartbeatPin,LOW);
  delay(100); // Should be enough time to pulse to get the 555 to recognize it
  digitalWrite(heartbeatPin, HIGH);
  // Return to high impedance
  pinMode(heartbeatPin, INPUT);
}

#if defined(COVERS)
void SaveCoverPos(int cover) {
    ShowDebug(F("Checking if we should save cover position:"));
    if (CoverPos[cover] != configuration.cover_pos[cover]) {
        ShowDebug(F("Saving cover position:"));
        configuration.cover_pos[cover] = CoverPos[cover];
        ShowDebug("Cover"+String(cover)+" : "+String(configuration.cover_pos[cover]));
        EEPROM_writeAnything(0, configuration);
      }
    else {
        ShowDebug(F("No save"));
    }
    CoverSetPos[cover] = 255; // and set coversetpos to unset position again
}
#endif

#if defined(LIGHTS)
void SetLightState(int light, String state) {
  if (state == "ON") {
    if (LightPins[light] < 100) {
        ShowDebug("Set light " + String(LightPins[light]) + " on.");
        digitalWrite(LightPins[light], !LightInitialState[light]);
    }
#if defined(MCP_present)
    else {
        ShowDebug("Set light " + String(LightPins[light] - 100) + " on.");
        mcp.digitalWrite(LightPins[light] - 100, !LightInitialState[light]);
    }
#endif    
  }
  else if (state == "OFF") {
    if (LightPins[light] < 100) {    
        ShowDebug("Set light " + String(LightPins[light]) + " off.");
        digitalWrite(LightPins[light], LightInitialState[light]);
        LightValue[light] = 0;
    }
#if defined(MCP_present)
    else {
        ShowDebug("Set light " + String(LightPins[light] - 100) + " off.");
        mcp.digitalWrite(LightPins[light] - 100, LightInitialState[light]);
        LightValue[light] = 0;
    }
#endif 
  }
  else {
    if (LightPins[light] < 100) {    
        ShowDebug("Setting pwm value " + String(LightValue[light]) + "on pin " + String(LightPins[light]));
        if (LightInitialState[light]) {
          analogWrite(LightPins[light], 255 - LightValue[light]);
        }
        else {
          analogWrite(LightPins[light], LightValue[light]);
        }
    }
#if defined(MCP_present)
// dunno how to handle pwm for mcp yet
#endif 
  }
  report_state_light(light);
}
#endif

void ProcessPulseRelays(int PulseRelayId) {
  // Process the timers of the pulse relays and see if we have to close them.
  if (digitalRead(PulseRelayPins[PulseRelayId]) == !PulseRelayInitialStates[PulseRelayId])
  {
    if ((millis() - PulseActivityTimes[PulseRelayId]) > PulseRelayTimes[PulseRelayId])
    {
      ShowDebug("Disabling pulse relay" + String(PulseRelayId) + ".");
      ShowDebug(String(PulseActivityTimes[PulseRelayId]));
      
//       String messageString = "P" + String(PulseRelayId) + "0";
//       messageString.toCharArray(messageBuffer, messageString.length() + 1);
//       mqttClient.publish(topic_out_pulse, messageBuffer);
      
      report_pulse_relay(PulseRelayId, false);
      
      digitalWrite(PulseRelayPins[PulseRelayId], PulseRelayInitialStates[PulseRelayId]);

#if defined(COVERS)
      // Update end position of covers
      for (int i = 0 ; i < NumberOfCovers ; i++) {
        ShowDebug("Checking status of cover " + String(i));
        ShowDebug("CoverPulse: " + String(CoverPulse[i]) + " - " + String(PulseRelayId));
        if (CoverPulse[i] == PulseRelayId) {
          if (CoverState[i] == 1) {   // 1 = opening
            CoverPos[i] = 100; // positon 100 = open
            CoverState[i] = 0;  // 0 = open
          }
          else {
            CoverPos[i] = 0;    // position 0 = closed
            CoverState[i] = 2;  // 2 = closed
          }
          ShowDebug(F("Setting cover position:"));
          ShowDebug(String(CoverPos[i]));
          SaveCoverPos(i);
        }
      } // end update covers
#endif

#if defined(LOCKS)
      // Update lock status
      for (int i = 0 ; i < NumberOfLocks ; i++) {
        if (LockPulse[i] == PulseRelayId) {
          if (LockState[i] == 0) {   // 0 = unlocked
            LockState[i] = 1;
            report_state_lock();
          }
        }
      } // end update lock status
#endif
    }

#if defined(COVERS)
    else { // Update current position of covers
      int LastPosition;
      for (int i = 0 ; i < NumberOfCovers ; i++) {
        if (CoverPulse[i] == PulseRelayId) {
          LastPosition = CoverPos[i];
          int progress = int(100 * (millis() - PulseActivityTimes[PulseRelayId]) / PulseRelayTimes[PulseRelayId]);
          if (CoverState[i] == 1) {     // 1 = opening, position goes from current position to 100
            CoverPos[i] = CoverStart[i] + progress;
          } else if (CoverState[i] == 3) {     // 3 = closing, position goes from current position to 0
            CoverPos[i] = CoverStart[i] - progress;
          }
          if (LastPosition != CoverPos[i]) {
            report_state_cover();
          }
        }
      } // end update covers
    }
#endif    
    
  }
}

#if defined(TRIGGERS)
void processButtonDigital( int buttonId ) {
    bool readButton;
    if (ButtonPins[buttonId] < 100) {
        readButton = digitalRead(ButtonPins[buttonId]);
    }
#if defined(MCP_present)
    else {
        readButton = mcp.digitalRead(ButtonPins[buttonId] - 100);
    }
#endif  
  if ( !readButton ) // Input pulled low to GND. Button pressed.
  {
    if ( !lastButtonStates[buttonId] )  // The button was previously un-pressed
    {
      if ((millis() - lastActivityTimes[buttonId]) > DEBOUNCE_DELAY) // Proceed if we haven't seen a recent event on this button
      {
        lastActivityTimes[buttonId] = millis();
      }
    }
    else if ((millis() - lastActivityTimes[buttonId] > LONGPRESS_TIME) && (!LongPressActive[buttonId]))// Button long press
    {
      LongPressActive[buttonId] = true;
      ShowDebug( "Button" + String(buttonId) + " long pressed" );
      String messageString = "Button" + String(buttonId) + "_long";
      messageString.toCharArray(messageBuffer, messageString.length() + 1);
      mqttClient.publish(state_topic_buttons, messageBuffer);
    }
    lastButtonStates[buttonId] = HIGH;
  }
  else {
    if (lastButtonStates[buttonId]) {
      if (LongPressActive[buttonId]) {
        LongPressActive[buttonId] = false;
      } 
      else {
        if ((millis() - lastActivityTimes[buttonId]) > DEBOUNCE_DELAY) // Proceed if we haven't seen a recent event on this button
        {
          ShowDebug( "Button" + String(buttonId) + " pressed" );
          String messageString = "Button" + String(buttonId);
          messageString.toCharArray(messageBuffer, messageString.length() + 1);
          mqttClient.publish(state_topic_buttons, messageBuffer);
        }
      }
      lastButtonStates[buttonId] = LOW;
    }
  }
}
#endif

void check_pir(byte pirid) {
  // ...read out the PIR sensors...
  bool readPir;
  if (PirSensors[pirid] < 100) {
     readPir = digitalRead(PirSensors[pirid]);
  }
#if defined(MCP_present)
  else {
     readPir = mcp.digitalRead(PirSensors[pirid] - 100);
  }
#endif  
  if (readPir != lastPirStates[pirid])
  {
    PirLastActivityTimes[pirid] = millis();
    lastPirStates[pirid] = readPir;
  }
  if (readPir != PirInitialState[pirid]) 
  {
    if (!PreviousDetects[pirid]) {
      if ((millis() - PirLastActivityTimes[pirid]) > PirDebounce[pirid])
      { 
          ShowDebug("Pir " + String(pirid) + " on.");
          PreviousDetects[pirid] = true;
          PirState[pirid] = 1;
          report_state_pir();
      }
    }
  }
  else {
    if (PreviousDetects[pirid]) {
      ShowDebug("Pir " + String(pirid) + " off.");
      PirState[pirid] = 0;
      report_state_pir();
    }
    PreviousDetects[pirid] = false;
  }
}

#if defined(MQ7_present)
float raw_value_to_CO_ppm(float value) {
  float reference_resistor_kOhm = 10.0;

  float sensor_reading_100_ppm_CO = -1;
  float sensor_reading_clean_air = 675;

  float sensor_100ppm_CO_resistance_kOhm;
  float sensor_base_resistance_kOhm;

  if (value < 1) return -1; //wrong input value
  sensor_base_resistance_kOhm = reference_resistor_kOhm * 1023 / sensor_reading_clean_air - reference_resistor_kOhm;
  if (sensor_reading_100_ppm_CO > 0)
  {
    sensor_100ppm_CO_resistance_kOhm = reference_resistor_kOhm * 1023 / sensor_reading_100_ppm_CO - reference_resistor_kOhm;
  }
  else
  {
    sensor_100ppm_CO_resistance_kOhm = sensor_base_resistance_kOhm * 0.25;
    //UPDATED: when checked on a CO meter, it seems that sensor corresponds to
    //the datasheet pretty well
  }
  float sensor_R_kOhm = reference_resistor_kOhm * 1023 / value - reference_resistor_kOhm;
  float R_relation = sensor_100ppm_CO_resistance_kOhm / sensor_R_kOhm;
  float CO_ppm = 134 * R_relation - 35;
  if (CO_ppm < 0) CO_ppm = 0;
  return CO_ppm;
}
#endif

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
#if defined(UNO_WIFI)
    set_rgb_led(64, 0, 0);  // RED
    status = WiFi.begin(ssid, pass);
    while ( status != WL_CONNECTED) {
    ShowDebug(F("Attempting to connect to WPA SSID: "));
    ShowDebug(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    }
#endif
    ShowDebug(F("Attempting MQTT connection..."));
    // Attempt to connect
    if (mqttClient.connect(CLIENT_ID, MQTT_USER, MQTT_PASS, status_topic, willQoS, willRetain, last_will)) {
      ShowDebug(F("Connected to MQTT broker."));
#if defined(UNO_WIFI)
      set_rgb_led(0, 0, 64);  // BLUE
#endif
      // Once connected, publish an announcement...
      mqttClient.publish(topic_out, ip.c_str());
      mqttClient.publish(topic_out, CLIENT_ID);
      // ... and resubscribe
      mqttClient.subscribe(topic_in);
    } else {
      ShowDebug("MQTT connection failed, rc=" + String(mqttClient.state()));
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void sendMessage(String m, char* topic) {
  m.toCharArray(messageBuffer, m.length() + 1);
  mqttClient.publish(topic, messageBuffer);
}

float CheckIsNan(float value, float defaultvalue) {
  if (isnan(value)) value = defaultvalue;
  return value;
}
float fmap(float value, float min, float max, float tmin, float tmax) {
  return (value - min) * (tmax - tmin) / (max - min) + tmin;
}

void sendData() {
  StaticJsonDocument<300> doc;
  float t, h, hic;
  for (int i = 0; i < NumberOfSensors; i++) {
    ShowDebug(String(i)+":"+SensorTypes[i]);
    if (SensorTypes[i] == "TIME") {
      doc["sensor" + String(i + 1)] = millis() / 1000;
    }
#if defined(DHT_present)
    else if (SensorTypes[i] == "DHT-T") {
      t = CheckIsNan(dht.readTemperature(), 0);
      doc["sensor" + String(i + 1)] = t;
    }
    else if (SensorTypes[i] == "DHT-H") {
      h = CheckIsNan(dht.readHumidity(), 0);
      doc["sensor" + String(i + 1)] = h;
    }
    else if (SensorTypes[i] == "DHT-I") {
      hic = CheckIsNan(dht.computeHeatIndex(t, h, false), -99);
      doc["sensor" + String(i + 1)] = hic;
    }
#endif
#if defined(LDR_present)
    else if (SensorTypes[i] == "LDR") {
      doc["sensor" + String(i + 1)] = fmap(analogRead(LightSensor), 0, 1023, 0, 100);
    }
#endif
#if defined(DS18B20_present)
    else if (SensorTypes[i] == "DS18B20") {
      t = sensors.getTempCByIndex(0);
      if (t < -50) { // fix for anomalous readings
        t = last_temp;
      }
      else if (t > 50) { // fix for anomalous readings
        t = last_temp;
      }
      else {
        last_temp = t;
      }
      doc["sensor" + String(i + 1)] = t;
      sensors.requestTemperatures();
    }
#endif
#if defined(BMP_present)
    else if (SensorTypes[i] == "BMP-T") {
      t = bmp.readTemperature();
      doc["sensor" + String(i + 1)] = t;
    }
    else if (SensorTypes[i] == "BMP-P") {
      t = bmp.readPressure() / 100;
      doc["sensor" + String(i + 1)] = t;
    }
#endif
#if defined(MS_present)
    else if (SensorTypes[i] == "MS") {
      doc["sensor" + String(i + 1)] = fmap(ms_value,0,1023,100,0);
    }
#endif
#if defined(MQ_present)
    else if (SensorTypes[i] == "MQ2") {
      doc["sensor" + String(i + 1)] = fmap(analogRead(SmokeSensor), 0, 1023, 0, 100);
    }
#endif
#if defined(MQ7_present)
    else if (SensorTypes[i] == "MQ7") {
      doc["sensor" + String(i + 1)] = raw_value_to_CO_ppm(co_value);
    }
#endif
#if defined(P1_meter)
    else if (SensorTypes[i] == "P1_en_t1") {
      if (last_p1_data.energy_delivered_tariff1.val() > 0) {
        ShowDebug("processing sensor "+ String(SensorTypes[i]));
        doc["sensor" + String(i + 1)] = last_p1_data.energy_delivered_tariff1.val();
      }
    }
    else if (SensorTypes[i] == "P1_en_t2") {
      if (last_p1_data.energy_delivered_tariff2.val() > 0) {
        ShowDebug("processing sensor "+ String(SensorTypes[i]));
        doc["sensor" + String(i + 1)] = last_p1_data.energy_delivered_tariff2.val();
      }
    }
    else if (SensorTypes[i] == "P1_rt_t1") {
      ShowDebug("processing sensor "+ String(SensorTypes[i]));
      if (last_p1_data.energy_returned_tariff1.val() > 0) {
        doc["sensor" + String(i + 1)] = last_p1_data.energy_returned_tariff1.val();
      }
    }
    else if (SensorTypes[i] == "P1_rt_t2") {
      ShowDebug("processing sensor "+ String(SensorTypes[i]));
      if (last_p1_data.energy_returned_tariff2.val() > 0) {
        doc["sensor" + String(i + 1)] = last_p1_data.energy_returned_tariff2.val();
      }
    }
    else if (SensorTypes[i] == "P1_ta") {
      if (last_p1_data.electricity_tariff.toInt() == 2) {
        doc["sensor" + String(i + 1)] = "hoog";
      }
      else {
        doc["sensor" + String(i + 1)] = "laag";
      }
    }
    else if (SensorTypes[i] == "P1_pd") {
      ShowDebug("processing sensor "+ String(SensorTypes[i]));
      doc["sensor" + String(i + 1)] = last_p1_data.power_delivered.int_val();
    }
    else if (SensorTypes[i] == "P1_pr") {
      ShowDebug("processing sensor "+ String(SensorTypes[i]));
      doc["sensor" + String(i + 1)] = last_p1_data.power_returned.int_val();
    }
    else if (SensorTypes[i] == "P1_v1") {
      ShowDebug("processing sensor "+ String(SensorTypes[i]));
      doc["sensor" + String(i + 1)] = last_p1_data.voltage_l1.val();
    }
    else if (SensorTypes[i] == "P1_c1") {
      ShowDebug("processing sensor "+ String(SensorTypes[i]));
      doc["sensor" + String(i + 1)] = last_p1_data.current_l1;
    }
    else if (SensorTypes[i] == "P1_pd1") {
      ShowDebug("processing sensor "+ String(SensorTypes[i]));
      doc["sensor" + String(i + 1)] = last_p1_data.power_delivered_l1.int_val();
    }
    else if (SensorTypes[i] == "P1_pr1") {
      ShowDebug("processing sensor "+ String(SensorTypes[i]));
      doc["sensor" + String(i + 1)] = last_p1_data.power_returned_l1.int_val();
    }
    else if (SensorTypes[i] == "P1_gas") {
      if (last_p1_data.gas_delivered.val() > 0) {
        ShowDebug("processing sensor "+ String(SensorTypes[i]));
        doc["sensor" + String(i + 1)] = last_p1_data.gas_delivered.val();
      }
    }
#endif
  }
  serializeJson(doc, messageBuffer);
  ShowDebug(F("Sending MQTT state for sensors..."));
//   serializeJsonPretty(doc, Serial);
//   ShowDebug(messageBuffer);
  mqttClient.publish(state_topic_sensors, messageBuffer);
}

#if defined(COVERS)
void OpenCover(int Cover) {
  int PulseRelayPort = CoverPulse[Cover];
  ShowDebug("Pulse relay index = " + String(PulseRelayPort));
  ShowDebug("Pulse relay port = " + String(PulseRelayPins[PulseRelayPort]));
  ShowDebug("Status port = " + String(digitalRead(PulseRelayPins[PulseRelayPort])));
  // Check if another command is already running
  if (digitalRead(PulseRelayPins[PulseRelayPort]) == PulseRelayInitialStates[PulseRelayPort]) {
    ShowDebug("Opening cover number " + String(Cover + 1));
    // Set direction relay
    digitalWrite(RelayPins[CoverDir[Cover]], !RelayInitialState[CoverDir[Cover]]);
    // Set opening pulse
    digitalWrite(PulseRelayPins[PulseRelayPort], !PulseRelayInitialStates[PulseRelayPort]);
    
//     String messageString = "P" + String(PulseRelayPort) + "1";
//     messageString.toCharArray(messageBuffer, messageString.length() + 1);
//     mqttClient.publish(topic_out_pulse, messageBuffer);
    
    report_pulse_relay(PulseRelayPort, true);
    
    ShowDebug("Cover: Starting pulse time on " + String(millis()));
    PulseActivityTimes[PulseRelayPort] = millis();
    // report state of screen
    CoverState[Cover] = 1; // 1 is opening
    CoverStart[Cover] = CoverPos[Cover];
    report_state_cover();
  }
  else {
    // Commmand given while cover moving, Stop pulse
    ShowDebug("Stopping cover number " + String(Cover));
    digitalWrite(PulseRelayPins[PulseRelayPort], PulseRelayInitialStates[PulseRelayPort]);
    
//     String messageString = "P" + String(PulseRelayPort) + "0";
//     messageString.toCharArray(messageBuffer, messageString.length() + 1);
//     mqttClient.publish(topic_out_pulse, messageBuffer);
    
    report_pulse_relay(PulseRelayPort, false);    
    
    SaveCoverPos(Cover);
  }
}
void CloseCover(int Cover) {
  int PulseRelayPort = CoverPulse[Cover];
  // Check if another command is already running
  if (digitalRead(PulseRelayPins[PulseRelayPort]) == PulseRelayInitialStates[PulseRelayPort]) {
    ShowDebug("Closing cover number " + String(Cover + 1));
    // Set direction relay
    digitalWrite(RelayPins[CoverDir[Cover]], RelayInitialState[CoverDir[Cover]]);
    // Set opening pulse
    digitalWrite(PulseRelayPins[PulseRelayPort], !PulseRelayInitialStates[PulseRelayPort]);
    
//     String messageString = "P" + String(PulseRelayPort) + "1";
//     messageString.toCharArray(messageBuffer, messageString.length() + 1);
//     mqttClient.publish(topic_out_pulse, messageBuffer);
    
    report_pulse_relay(PulseRelayPort, true);    
    
    ShowDebug("Cover: Starting pulse time on " + String(millis()));
    PulseActivityTimes[PulseRelayPort] = millis();
    // report state of screen
    CoverState[Cover] = 3;  // 3 is closing
    CoverStart[Cover] = CoverPos[Cover];
    report_state_cover();
  }
  else {
    ShowDebug("Stopping cover number " + String(Cover + 1));
    // Commmand given while cover moving, Stop pulse
    digitalWrite(PulseRelayPins[PulseRelayPort], PulseRelayInitialStates[PulseRelayPort]);
    
//     String messageString = "P" + String(PulseRelayPort) + "0";
//     messageString.toCharArray(messageBuffer, messageString.length() + 1);
//     mqttClient.publish(topic_out_pulse, messageBuffer);
//     
    report_pulse_relay(PulseRelayPort, false);    
    
    SaveCoverPos(Cover);
  }
}
void StopCover(int Cover) {
  if ((CoverState[Cover] & 1) == 1) {
    int PulseRelayPort = CoverPulse[Cover];
    // Stop pulse
    ShowDebug("Stop cover number " + String(Cover + 1));
    digitalWrite(PulseRelayPins[PulseRelayPort], PulseRelayInitialStates[PulseRelayPort]);
    
//     String messageString = "P" + String(PulseRelayPort) + "0";
//     messageString.toCharArray(messageBuffer, messageString.length() + 1);
//     mqttClient.publish(topic_out_pulse, messageBuffer);
    
    report_pulse_relay(PulseRelayPort, false);    
    
  }
  if (CoverPos[Cover] <= 0) {
    CoverPos[Cover] = 0;
    CoverState[Cover] = 2;
  }
  else if (CoverPos[Cover] >= 100) {
    CoverPos[Cover] = 100;
    CoverState[Cover] = 0;
  }
  else {
    CoverState[Cover] = 4;
  }
  SaveCoverPos(Cover);
}
void SetCoverPosition(int cover, int position) {
  CoverSetPos[cover] = position;
  if (CoverPos[cover] < position) {
    ShowDebug(F("Cover needs to open"));
    if (position == 100) {
      CoverSetPos[cover] = 120;
    }
    CoverState[cover] = 1; // opening
    OpenCover(cover);
  }
  else {
    ShowDebug(F("Cover needs to close"));
    if (position == 0) {
      CoverSetPos[cover] = -20;
    }
    CoverState[cover] = 3; // closing
    CloseCover(cover);
  }
}

void ProcessCovers(int cover) {
  if (CoverPos[cover] == CoverSetPos[cover]) {
    ShowDebug(F("ProcessCovers"));
    StopCover(cover);
    SaveCoverPos(cover);
  }
}
#endif

void onMessage(char* topic, byte * payload, byte length) {
  StaticJsonDocument<20> doc;
//   char msgBuffer[BUFFERSIZE];
  payload[length] = '\0'; // terminate string with 0
  String strPayload = String((char*)payload);  // convert to string
  ShowDebug(F("Message arrived"));
  ShowDebug(topic);
  ShowDebug(strPayload);

  byte RelayPort;
  byte RelayValue;
  
#if defined(LIGHTS)
  if (String(topic).indexOf(cmd_topic_lights) >= 0) {
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
#endif  

  if (strPayload[0] == '{') {
    // json message
    deserializeJson(doc, strPayload);
#if defined(COVERS)
    for (int i = 0; i < NumberOfCovers; i++) {
      String keyword = "POSITION" + String(i + 1);
      ShowDebug("Keyword: " + String(keyword));
      if (doc.containsKey(keyword)) {
        ShowDebug("received position for cover #" + String(i + 1) + " :");
        ShowDebug(doc[keyword]);
        SetCoverPosition(i, doc[keyword]);
      }
    }
#endif
  }
  else if (strPayload[0] == 'R') {

    // Relais commando
    ShowDebug(F("Relay command:"));
    
    ShowDebug("Length=" + String(strPayload.length()));

    if (strPayload.length() > 3) {
        ShowDebug(strPayload.substring(1,3));
        RelayPort = strPayload.substring(1,3).toInt();
        RelayValue = strPayload[3] - 48;
    }
    else {
        RelayPort = strPayload[1] - 48;
        RelayValue = strPayload[2] - 48;
    }

    if (RelayPort > 16) RelayPort -= 3;
    ShowDebug("Relay " + String(RelayPins[RelayPort]));
    if (RelayValue == 40) { // toggle
      if (RelayPins[RelayPort] < 100) {
          
      
          if (digitalRead(RelayPins[RelayPort]) == LOW) {
            digitalWrite(RelayPins[RelayPort], HIGH);
            ShowDebug(F("...to HIGH"));
          }
          else {
            digitalWrite(RelayPins[RelayPort], LOW);
            ShowDebug(F("...to LOW"));
          }
      }
#if defined(MCP_present)
      else {
          ShowDebug("Relay " + String (RelayPins[RelayPort] - 100));     
          if (mcp.digitalRead(RelayPins[RelayPort] - 100) == LOW) {
            mcp.digitalWrite(RelayPins[RelayPort] - 100, HIGH);
            ShowDebug(F("...to HIGH"));
          }
          else {
            mcp.digitalWrite(RelayPins[RelayPort] - 100, LOW);
            ShowDebug(F("...to LOW"));
          }
      }
#endif
    } 
    else {
        ShowDebug("...to " + String(RelayValue));
        if (RelayPins[RelayPort] < 100) {
             digitalWrite(RelayPins[RelayPort], RelayValue);
             
          }
#if defined(MCP_present)
        else {
          mcp.digitalWrite(RelayPins[RelayPort] - 100, RelayValue);
        }
#endif
        }
    report_state_relay();
  } 
  else if (strPayload == "IP")  {

    // 'Show IP' commando
    mqttClient.publish(topic_out, ip.c_str());// publish IP nr
  }
  else if (strPayload == "AON") {

    // Alle relais aan
    for (byte i = 0 ; i < NumberOfRelays; i++) {
      if (RelayInitialState[i] == LOW) {
        digitalWrite(RelayPins[i], HIGH);
      } else {
        digitalWrite(RelayPins[i], LOW);
      }
    }
    report_state_relay();
  }
  else if (strPayload == "AOF") {
    // Alle relais uit
    for (byte i = 0 ; i < NumberOfRelays; i++) {
      digitalWrite(RelayPins[i], RelayInitialState[i]);
    }
    report_state_relay();
  }
  else if (strPayload == "STAT") {

    // Status van alle sensors and relais
    sendData();
  }
  else if (strPayload == "SAVE") {

#if defined(COVERS)
    // Save current parameters
    for (int id = 0; id < NumberOfCovers; id++) {
        configuration.cover_pos[id] = CoverPos[id];
        configuration.saved = true;
    }
    EEPROM_writeAnything(0, configuration);
#endif
  }
  else if (strPayload == "#RESET") {
    ShowDebug(F("Reset command received, resetting in one second..."));
    delay(1000);
    resetFunc();
  }
  else if (strPayload == "#DEBUGON") {
    ShowDebug(F("Debug on"));
    debug = true;
    ShowDebug("Client ID: "+String(CLIENT_ID));
    ShowDebug("MQTT Packet Size: "+String(MQTT_MAX_PACKET_SIZE));
  }
  else if (strPayload == "#DEBUGOFF") {
    ShowDebug(F("Debug off"));
    debug = false;
  }
  else if (strPayload[0] == 'P') {  // PULSE RELAY
    int PulseRelayPort = strPayload[1] - 48;
    if (PulseRelayPort < NumberOfPulseRelays) {
      if (strPayload[2] == '1') {  // Pulserelay on
        ShowDebug("Enabling pulse relay " + String(PulseRelayPort) + ".");
        digitalWrite(PulseRelayPins[PulseRelayPort], !PulseRelayInitialStates[PulseRelayPort]);
        
//         String messageString = "P" + String(PulseRelayPort) + "1";
//         messageString.toCharArray(messageBuffer, messageString.length() + 1);
//         mqttClient.publish(topic_out_pulse, messageBuffer);
        
        report_pulse_relay(PulseRelayPort, true);    
        
        PulseActivityTimes[PulseRelayPort] = millis();
        ShowDebug("Starting pulse time on " + String(millis()));
      }
      else { // Pulserelay forced off
        ShowDebug("Disabling pulse relay " + String(PulseRelayPort) + ".");
        
//         String messageString = "P" + String(PulseRelayPort) + "0";
//         messageString.toCharArray(messageBuffer, messageString.length() + 1);
//         mqttClient.publish(topic_out_pulse, messageBuffer);

        report_pulse_relay(PulseRelayPort, false);    
        
        digitalWrite(PulseRelayPins[PulseRelayPort], PulseRelayInitialStates[PulseRelayPort]);
      }
    }
    else {
      ShowDebug(F("No such pulserelay defined!"));
    }
  }
  // *** Control for screen covers ****
  // Command: O + cover number [+ pulse duration in ms]
  //  opens cover:
  //    - sets direction relay: open = inverse state
  //    - pulse duration taken from command, if not given, use parameter COVERDELAYTIME
  //    - opens motor relay for duration of pulse
#if defined(COVERS)
  else if (strPayload[0] == 'O') {
    // Cover commando: open
    ShowDebug(F("Cover command : Open"));
    byte CoverPort = strPayload[1] - 48;
    // 2nd char is # of cover
    ShowDebug("Cover number " + String(CoverPort));
    if (CoverPort <= NumberOfCovers) {
      SetCoverPosition(CoverPort - 1, 100);
    }
    else {
      ShowDebug("No such cover defined: " + String(CoverPort));
    }
  } // End of Cover commando: open
  //
  // Command: C + cover number [+ pulse duration in ms]
  //  closes cover:
  //    - sets direction relay: open = normal state
  //    - pulse duration taken from command, if not given, use parameter COVERDELAYTIME
  //    - opens motor relay for duration of pulse
  //
  else if (strPayload[0] == 'C') {
    // Cover commando: close
    ShowDebug(F("Cover command : Close"));
    byte CoverPort = strPayload[1] - 48;
    ShowDebug("Cover number " + String(CoverPort));
    if (CoverPort <= NumberOfCovers) {
      SetCoverPosition(CoverPort - 1, 0);
    }
    else {
      ShowDebug("No such cover defined: " + String(CoverPort));
    }
  }
  //
  // Command: S + cover number
  //  stops cover movement:
  //    - closes motor relay, stops pulse
  //
  else if (strPayload[0] == 'S') {
    // Cover commando: stop
    ShowDebug(F("Cover command : Stop"));
    byte CoverPort = strPayload[1] - 48;
    ShowDebug("Cover number " + String(CoverPort));
    if (CoverPort <= NumberOfCovers) {
      StopCover(CoverPort - 1);
    }
    else {
      ShowDebug("No such cover defined: " + String(CoverPort));
    }
  }
#endif
#if defined(LOCKS)
  // MQTT Discovery Lock commands
  else if (strPayload[0] == 'L') {
    // Lock commando: lock
    ShowDebug(F("Lock command : lock"));
    byte LockPort = strPayload[1] - 48;
    ShowDebug("Lock number " + String(LockPort));
    int PulseRelayPort = LockPulse[LockPort - 1];
    if (LockPort <= NumberOfLocks) {
      // End opening pulse
      digitalWrite(PulseRelayPins[PulseRelayPort], PulseRelayInitialStates[PulseRelayPort]);
      
//       String messageString = "P" + String(PulseRelayPort) + "0";
//       messageString.toCharArray(messageBuffer, messageString.length() + 1);
//       mqttClient.publish(topic_out_pulse, messageBuffer);
      
      report_pulse_relay(PulseRelayPort, false);    
      
      LockState[LockPort - 1] = 1;
      report_state_lock();
    }
    else {
      ShowDebug("No such lock defined: " + String(LockPort));
    }
  }
  else if (strPayload[0] == 'U') {
    // Lock commando: unlock
    ShowDebug(F("Lock command : unlock"));
    byte LockPort = strPayload[1] - 48;
    ShowDebug("Lock number " + String(LockPort));
    int PulseRelayPort = LockPulse[LockPort - 1];
    if (LockPort <= NumberOfLocks) {
      // Set opening pulse
      digitalWrite(PulseRelayPins[PulseRelayPort], !PulseRelayInitialStates[PulseRelayPort]);
      
//       String messageString = "P" + String(PulseRelayPort) + "1";
//       messageString.toCharArray(messageBuffer, messageString.length() + 1);
//       mqttClient.publish(topic_out_pulse, messageBuffer);

      report_pulse_relay(PulseRelayPort, true);    
      
      PulseActivityTimes[PulseRelayPort] = millis();
      LockState[LockPort - 1] = 0;
      report_state_lock();
    }
    else {
      ShowDebug("No such lock defined: " + String(LockPort));
    }
  }
#endif
  else {
    // Onbekend commando
    ShowDebug(F("Unknown value"));
    mqttClient.publish(topic_out, "Unknown command");
  }
}
void setDeviceInfo(char* configtopic, StaticJsonDocument<512> doc) {
  JsonObject device = doc.createNestedObject("dev");
  JsonArray identifiers = device.createNestedArray("ids");
  identifiers.add(CLIENT_ID);
  JsonArray connections = device.createNestedArray("cns");
  ShowDebug(String(ip));
  connections.add(serialized("[\"ip\",\"" + String(ip) + "\"]"));
#if !defined(UNO_WIFI)
  ShowDebug(mac2String(mac));
  connections.add(serialized("[\"mac\",\"" + mac2String(mac) + "\"]"));
#endif
  device["name"] = DISCOVERY_ID;
  device["mdl"] = MODEL_ID;
  device["mf"] = MANUFACTURER_ID;
  device["sw"] = version;
  size_t n = serializeJson(doc, messageBuffer);
  ShowDebug(F("Sending MQTT config on topic:"));
  ShowDebug(configtopic);
  ShowDebug(F("Config json:"));
  ShowDebug(messageBuffer);
  if (mqttClient.publish(configtopic, messageBuffer, n)) {
    ShowDebug(F("...MQTT config sent."));
  }
  else {
    ShowDebug(F("...publish failed, either connection lost, or message too large."));
  }
}

void reportMQTTdisco() {
  StaticJsonDocument<512> doc;
  // discovery data for relays
  for (int i = 0; i < NumberOfRelays ; i++) {
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
    setDeviceInfo((config_topic_base + "/switch/" + item_prefix + "_switch" + String(i + 1) + "/config").c_str(), doc);
  }
  // discovery data for pulse relays
  for (int i = 0; i < NumberOfPulseRelays ; i++) {
    doc.clear();
    doc["name"] = PulseSwitchNames[i];
    doc["uniq_id"] = item_prefix + "_pulseswitch" + String(i + 1);
    doc["stat_t"] = state_topic_pulserelays;
    doc["cmd_t"] = topic_in;
    doc["pl_on"] = "P" + String(i) + "1";
    doc["stat_on"] = "on";
    doc["stat_off"] = "off";
    doc["val_tpl"] = "{{value_json.PULSE" + String(i) + "}}";
    setDeviceInfo((config_topic_base + "/switch/" + item_prefix + "_pulseswitch" + String(i + 1) + "/config").c_str(), doc);
  }  
#if defined(LIGHTS)
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
    setDeviceInfo((config_topic_base + "/light/" + item_prefix + "_light" + String(i + 1) + "/config").c_str(), doc);
  }
#endif
#if defined(COVERS)
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
    setDeviceInfo((config_topic_base + "/cover/" + item_prefix + "_cover" + String(i + 1) + "/config").c_str(), doc);
  }
#endif
#if defined(LOCKS)
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
    setDeviceInfo((config_topic_base + "/lock/" + item_prefix + "_lock" + String(i + 1) + "/config").c_str(), doc);
  }
#endif
  // discovery data for status message (binary_sensor)
  doc.clear();
  doc["name"] = "Status";
  doc["uniq_id"] = item_prefix + "_state";
  doc["stat_t"] = status_topic;
  doc["dev_cla"] = "problem";
  doc["pl_on"] = "error";
  doc["pl_off"] = "ok";
  setDeviceInfo((config_topic_base + "/binary_sensor/" + item_prefix + "_status" + "/config").c_str(), doc);
  
  // discover data for pirs (binary_sensors)
  for (int i = 0; i < NumberOfPirs ; i++ ) {
    doc.clear();
    doc["name"] = PirNames[i];
    doc["uniq_id"] = item_prefix + "_pir" + String(i + 1);
    doc["stat_t"] = state_topic_pirs;
    doc["dev_cla"] = PirClasses[i];
    doc["pl_on"] = "ON";
    doc["pl_off"] = "OFF";
    doc["val_tpl"] = " {{value_json.PIR" + String(i + 1) + "}}";
    setDeviceInfo((config_topic_base + "/binary_sensor/" + item_prefix + "_pir" + String(i + 1) + "/config").c_str(), doc);
  }
#if defined(TRIGGERS)
  // discover data for buttons (triggers)
   for (int i = 0; i < NumberOfButtons ; i++ ) {
     doc.clear();
     doc["automation_type"] = "trigger";
     doc["topic"] = state_topic_buttons;
     doc["type"] = "button_short_release";
     doc["subtype"] = "button_" + String(i+1);
     doc["payload"] = "Button" + String(i);
     setDeviceInfo((config_topic_base + "/device_automation/" + item_prefix + "_button" + String(i + 1) + "/config").c_str(), doc);
     doc.clear();
     doc["automation_type"] = "trigger";
     doc["topic"] = state_topic_buttons;
     doc["type"] = "button_long_press";
     doc["subtype"] = "button_" + String(i+1);
     doc["payload"] = "Button" + String(i) + "_long";
     setDeviceInfo((config_topic_base + "/device_automation/" + item_prefix + "_button" + String(i + 1) + "_long" + "/config").c_str(), doc);
   }
#endif
  // discover data for sensors (sensors)
  for (int i = 0; i < NumberOfSensors ; i++ ) {
    doc.clear();
    doc["name"] = SensorNames[i];
    doc["uniq_id"] = item_prefix + "_sensor" + String(i + 1);
    doc["stat_t"] = state_topic_sensors;
    if (SensorClasses[i] != "") {
      doc["dev_cla"] = SensorClasses[i];
    }
    if (StateClasses[i] != "") {
      doc["stat_cla"] = StateClasses[i];
    }
    if (SensorUnits[i] != "") {
      doc["unit_of_meas"] = SensorUnits[i];
    }
    if (SensorTypes[i] == "P1_ta"){
      doc["val_tpl"] = " {{value_json.sensor" + String(i + 1) + " }}";
    }
    else {
      doc["val_tpl"] = " {{value_json.sensor" + String(i + 1) + " | round (1) }}";
    }
    setDeviceInfo((config_topic_base + "/sensor/" + item_prefix + "_sensor"  + String(i + 1) + "/config").c_str(), doc);
  }
  //  end send config data for MQTT discovery
}
void setup() {
  
  version += "(";
  version += DOMUS_LIBRARY_VERSION;
  version += ")";

  if (debug) {
    ShowDebug("Client ID: "+String(CLIENT_ID));
    ShowDebug("MQTT Packet Size: "+String(MQTT_MAX_PACKET_SIZE));
  }
#if defined(COVERS)
  EEPROM_readAnything(0, configuration);
  ShowDebug(F("Reading stored configuration"));
  if (configuration.saved) {
      for (int id = 0; id < NumberOfCovers; id++) {
        CoverPos[id] = configuration.cover_pos[id];
        ShowDebug("Cover position: "+String(configuration.cover_pos[id]));
      }
  }
#endif
#if defined(MCP_present)
  Wire.setClock(400000);
  mcp.begin();                      // use default address 0 for i2c expander (MCP23017/MCP23008)
  Wire.beginTransmission(32);
  if (Wire.endTransmission () == 0) {
      ShowDebug(F("MCP23017 found at address 20h"));
  }
  else {
      ShowDebug(F("MCP23017 not found at address 20h !!"));
  }
#endif

  for (byte thisPin = 0; thisPin < NumberOfRelays; thisPin++) {
    ShowDebug("Relay: " + String(RelayPins[thisPin]) + " (" + String(SwitchNames[thisPin]) + ")");
    if (RelayPins[thisPin] < 100) { // pins > 100 are MCP ports
      pinMode(RelayPins[thisPin], OUTPUT);
      digitalWrite(RelayPins[thisPin], RelayInitialState[thisPin]);
    }
#if defined(MCP_present)
    else {
      mcp.pinMode(RelayPins[thisPin] - 100, OUTPUT);
      mcp.digitalWrite(RelayPins[thisPin] - 100, RelayInitialState[thisPin]);
    }
#endif
  }

#if defined(LIGHTS)
  for (byte thisPin = 0; thisPin < NumberOfLights; thisPin++) {
    ShowDebug("Light: " + String(LightPins[thisPin])+ " (" + String(LightNames[thisPin]) + ")");
    if (LightPins[thisPin] < 100) { // pins > 100 are MCP ports
      pinMode(LightPins[thisPin], OUTPUT);
      digitalWrite(LightPins[thisPin], LightInitialState[thisPin]);
    }
#if defined(MCP_present)    
    else {
      mcp.pinMode(LightPins[thisPin] - 100, OUTPUT);
      mcp.digitalWrite(LightPins[thisPin] - 100, LightInitialState[thisPin]);
    }
#endif
  }
#endif

  for (int thisPin = 0; thisPin < NumberOfPulseRelays; thisPin++) {
    ShowDebug("Pulse relay: " + String(PulseRelayPins[thisPin]));
    if (PulseRelayPins[thisPin] < 100) { // pins > 100 are MCP ports    
      pinMode(PulseRelayPins[thisPin], OUTPUT);
      digitalWrite(PulseRelayPins[thisPin], PulseRelayInitialStates[thisPin]);
    }
#if defined(MCP_present)  
    else {
      mcp.pinMode(PulseRelayPins[thisPin] - 100, OUTPUT);
      mcp.digitalWrite(PulseRelayPins[thisPin] - 100, PulseRelayInitialStates[thisPin]);
    }
#endif
  }

#if defined(TRIGGERS)
  for (int thisButton = 0; thisButton < NumberOfButtons; thisButton++) {
    ShowDebug("Button: " + String(ButtonPins[thisButton]));
    if (ButtonPins[thisButton] < 100) { // pins > 100 are MCP ports 
      pinMode(ButtonPins[thisButton], INPUT_PULLUP);
    }
#if defined(MCP_present)
    else {
      mcp.pinMode(ButtonPins[thisButton] - 100, INPUT);
      mcp.pullUp(ButtonPins[thisButton] - 100, HIGH);
    } 
#endif 
  }
#endif

  for (byte pirid = 0; pirid < NumberOfPirs; pirid++) {
    ShowDebug("Pir: " + String(PirSensors[pirid])+ " (" + String(PirNames[pirid]) + ")");
    lastPirStates[pirid] = PirInitialState[pirid];
    if (PirSensors[pirid] < 100) { // pins > 100 are MCP ports 
      pinMode(PirSensors[pirid], INPUT_PULLUP);
    }
#if defined(MCP_present)
    else {
      mcp.pinMode(PirSensors[pirid] - 100, INPUT);
      mcp.pullUp(PirSensors[pirid] - 100, HIGH);
    }
#endif
  }

#if defined(DHT_present)
  dht.begin(110); // 55 us is default
  ShowDebug(F("DHT-22 sensor: 3"));
#endif

#if defined(LDR_present)
  pinMode(LightSensor, INPUT);
  ShowDebug(F("LDR sensor: A10"));
#endif

#if defined(UNO_WIFI)
  analogReference(VDD);
#else
  analogReference(DEFAULT);
#endif

#if defined(MS_present)
  pinMode(MS_ENABLE, OUTPUT); // vul hier de enable pin in
#endif

#if defined(DS18B20_present)
  ShowDebug(F("DS18B20 sensor: 16"));
#endif

#if defined(BMP_present)
  if (!bmp.begin()) {
    ShowDebug(F("Could not find a valid BMP280 sensor, check wiring!"));
  }
  else {
    ShowDebug(F("BMP280 sensor (i2c): 20 (SDA), 21 (SCL)"));
  }
#endif

#if defined(MQ_present)
  pinMode(SmokeSensor, INPUT);
  ShowDebug("MQ2 sensor: " + String(SmokeSensor));
#endif

#if defined(MQ7_present)
  mq_millis = millis();
  ShowDebug("MQ7 sensor: " + String(mq_sensor_pin));
#endif

#if defined(P1_meter)
  Serial1.begin(115200);                // P1 meter connected to pin 19 (RX1) of Arduino Mega
  reader.enable(true);
  last_p1_read = millis();
  ShowDebug(F("Slimme meter via P1 geactiveerd via RX1 (pin19) "));
#endif

#if defined(UNO_WIFI)
    WiFiDrv::pinMode(25, OUTPUT);       //RED
    WiFiDrv::pinMode(26, OUTPUT);       //GREEN
    WiFiDrv::pinMode(27, OUTPUT);       //BLUE
    set_rgb_led(64, 64, 64);            // Set Wifi Status LED to WHITE
    while ( status != WL_CONNECTED) {
        ShowDebug(F("Attempting to connect to WPA SSID: "));
        ShowDebug(ssid);
        // Connect to WPA/WPA2 network:
        status = WiFi.begin(ssid, pass);
    }
    set_rgb_led(0, 64, 0);              // GREEN
    if (debug) {
        ip = String (WiFi.localIP()[0]);
        ip = ip + ".";
        ip = ip + String (WiFi.localIP()[1]);
        ip = ip + ".";
        ip = ip + String (WiFi.localIP()[2]);
        ip = ip + ".";
        ip = ip + String (WiFi.localIP()[3]); 
        ShowDebug(ip);
        ShowDebug("");   
    }
#else
    ShowDebug(F("Network..."));
    // attempt to connect to network:
    // setup ethernet communication using DHCP
    if (Ethernet.begin(mac) == 0) {
        ShowDebug(F("No DHCP"));
        delay(1000);
        resetFunc();
    }
    ShowDebug(F("Ethernet via DHCP"));
    ShowDebug(F("IP address: "));
    ip = String (Ethernet.localIP()[0]);
    ip = ip + ".";
    ip = ip + String (Ethernet.localIP()[1]);
    ip = ip + ".";
    ip = ip + String (Ethernet.localIP()[2]);
    ip = ip + ".";
    ip = ip + String (Ethernet.localIP()[3]);
    ShowDebug(ip);
    ShowDebug("");
#endif

  // setup mqtt client
  mqttClient.setBufferSize(BUFFERSIZE);
  mqttClient.setClient(NetClient);
  mqttClient.setServer(MQTTSERVER, 1883); // or local broker
  ShowDebug(F("MQTT set up"));
  mqttClient.setCallback(onMessage);
  ShowDebug(F("Ready to send data"));
  lastPublishTime = millis();
}

#if defined(P1_meter)
void PrintValues(MyData data) {
}
#endif

void loop() {
  // Main loop, where we check if we're connected to MQTT...
  if (!mqttClient.connected()) {
    ShowDebug(F("Not Connected!"));
//     resetFunc();
    reconnect();
    startsend = true;
  }

  // ... then send all relay stats and discovery info when we've just started up....
  if (startsend) {
    reportMQTTdisco();
    report_state();
    // send heartbeat
    heartbeat();
    startsend = false;
  }

  // ...handle the PulseRelays, ...
  for (int id = 0; id < NumberOfPulseRelays; id++) {
    ProcessPulseRelays(id);
  }
#if defined(COVERS)
  // handle the cover position
  for (int id = 0; id < NumberOfCovers; id++) {
    ProcessCovers(id);
  }
#endif
  // ...read out the PIR sensors...
  for (int id = 0; id < NumberOfPirs; id++) {
    check_pir(id);
  }

#if defined(MQ7_present)
  // ...process the MQ-7 sensor...
  if (mq_state == 0) {
    digitalWrite(mq_state_pin, HIGH);
    if ( mq_millis < millis() ) {
      ShowDebug(F("MQ-7 measure..."));
      mq_state = 1;
      mq_millis = millis() + mq_measure_interval;
    }
  }
  else {
    digitalWrite(mq_state_pin, LOW);
    int mq_value = analogRead(mq_sensor_pin);
    //    ShowDebug("Value: "+String(mq_value));
    if ( (mq_millis + mq_startup - mq_measure_interval) < millis() )
    {
      co_value *= 0.999;
      co_value += 0.001 * mq_value;
    }
    if ( mq_millis < millis() ) {
      ShowDebug(F("MQ-7 heatup..."));
      mq_state = 0;
      mq_millis = millis() + mq_heat_interval;
    }
  }
#endif

#if defined(MS_present)
  // ...process the MS sensor...
  if ( ms_measure_millis < millis() ) {
      if (ms_state == 0) {
        digitalWrite(MS_ENABLE, HIGH);
        ms_state = 1;
        ms_millis = millis() + ms_heat_interval;
      }
      else {
        if ( ms_millis < millis() ) {
           ShowDebug(F("MS measure..."));
           float f;
           ms_value = 0;
           for (int i = 0; i < SAMPLESIZE; i++) {
               // g += analogRead(MS_PIN);
               f = analogRead(MS_PIN);
               ShowDebug(String(f));
               ms_value += f;
           }
           ms_value /= SAMPLESIZE;
           digitalWrite(MS_ENABLE, LOW);
           ms_state = 0;
           ms_measure_millis = millis() + PUBLISH_DELAY;
        }
      }
  }
#endif

#if defined(P1_meter)
  reader.loop();
  unsigned long now = millis();
  if (now - last_p1_read > READER_INTERVAL) {
    reader.enable(true);
    last_p1_read = now;
  }
  if (reader.available()) {
    MyData data;
    String err;
    if (reader.parse(&data, &err)) {
      // Parse succesful, print result
      last_p1_data = data;
      if (debug) {
        PrintValues(data);
        }
    } else {
      // Parser error, print error
      ShowDebug(err);
    }
  }
#endif

  // ...see if it's time to send new data, ....
  if (millis() - lastPublishTime > PUBLISH_DELAY) {
    lastPublishTime = millis();
    sendData();
    heartbeat();
    ShowDebug("Free memory: "+String(freeMemory()));
  }
  else if (millis() - lastReportTime > REPORT_DELAY) {
    lastReportTime = millis();
    report_state();
    // send a heartbeat pulse
    heartbeat();
  }
#if defined(TRIGGERS)  
  else {
    for (int id = 0; id < NumberOfButtons; id++) {
      processButtonDigital(id);
    }
  }
#endif

  // and loop.
//   mqttClient.loop();

#if defined(RECYCLE)
  if (millis() > MaxUptime) resetFunc();
#endif
  
  if(!mqttClient.loop()) resetFunc();
}
