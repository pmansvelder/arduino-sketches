/*
          <========Arduino Sketch for Arduino Mega =========>
          Locatie: Schuur
          Macadres: 00:01:02:03:04:09
          Aansluiting via UTP kabel:
          - 5V : bruin
          - GND: wit

          Pins used:
          0: Serial
          1: Serial
          2:
          3: DHT-22
          4: <in gebruik voor W5100>
          5: Relais 0
          6: Relais 1
          7: Relais 2 : Lamp brandgang
          8: Relais 3 : Tuinlamp achter
          9: Relais 4 : Lamp Fietsenhok
          10: <in gebruik voor W5100>
          11: Relais 5
          12: Relais 6
          13: Relais 7 : Lamp Schuur

          20: SDA voor i2C
          21: SCL voor i2C
          22: PIR0
          23: PIR1
          24: PIR2
          25: PIR3
          30: Button0
          31: Button1
          32: PWM output for leds

          50: <in gebruik voor W5100>
          51: <in gebruik voor W5100>
          52: <in gebruik voor W5100>
          53: <in gebruik voor W5100>

          incoming topic: domus/schuur/in

          Arduino Mega with W5100 ethernet shield used as MQTT client
          It will connect over Ethernet to the MQTT broker and controls digital outputs (LED, relays)
          The topics have the format "domus/hobby/uit" for outgoing messages and
          "domus/schuur/in" for incoming messages.

          The outgoing topics are

          domus/schuur/uit        // Relaisuitgangen: R<relaisnummer><status>

          Here, each relay state is reported using the same syntax as the switch command:
          R<relay number><state>

          There is only one incoming topic:
          domus/schuur/in
          The payload here determines the action:
          STAT - Report the status of all relays (0-9)
          AON - turn all the leds on
          AOF - turn all the leds off
          2 - Publish the IP number of the client
          R<relay number><state> - switch led into specified state (0=off, 1=on)
          R<relay number>X - toggle relay

          On Startup, the Client publishes the IP number

          Adapted 4-2-2018 by Peter Mansvelder:

          removed Temp/Humidity, added multiple relays for MQTT house control
          I used the following ports:

          Uno: pins 4,10,11,12,13 in use
          Mega: 4,10,50,51,52,53 in use

          3,5,6,7,8,9,A0(14),A1(15),A2(16),A3(17), using those not used by ethernet shield (4, 10, 11, 12, 13) and other
          ports (0, 1 used by serial interface).
          A4(18) and A5(19) are used as inputs, for 2 buttons

          N.B.: changes to be made if sketch is used in production:
          - Change CLIENT_ID -> done
          - change Mac Address -> done
          - change DISCOVERY ID -> done
          - Change topic base from domus/hobby with find/replace -> done
          - Change item names
          - Change pin numbers for relays, buttons, pirs
          - Change pin numbers for sensors
          - item_prefix variable -> done


*/

#define BUFFERSIZE 512          // default 100
#define MQTT_MAX_PACKET_SIZE 512

#include <Ethernet.h>           // Ethernet.h library
#include "PubSubClient.h"       //PubSubClient.h Library from Knolleary, must be adapted: #define MQTT_MAX_PACKET_SIZE 512
#include "ArduinoJson.h"

StaticJsonDocument<512> doc;

#include "secrets.h"

// parameters to tune memory use
//#define BMP_present 1 // use BMP280 sensor
#define DHT_present 1 // use DHT sensor
//#define MQ_present 0 // MQ-x gas sensor
////#define MQ7_present 0 // MQ-7 CO sensor
//#define DS18B20_present 1 // DS18B20 1-wire temperature sensor
//#define LDR_present 1 // LDR sensor
#define DEBUG 1 // Zet debug mode aan

#if defined(DHT_present)
#include <DHT.h>
#define DHT_PIN 3 // Vul hier de pin in van de DHT11/22 sensor
DHT dht(DHT_PIN, DHT22);
#endif

#if defined(LDR_present)
int LightSensor = A10;
#endif

// DS18B20 sensor
#if defined(DS18B20_present)
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 16
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float last_temp = 0;
#endif

// BMP280 pressure and temperature sensor
#if defined(BMP_present)
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>    // Adafruit BMP280 library
Adafruit_BMP280 bmp; // I2C: SDA=20, SCL=21
#endif

#if defined(MQ_present)
int SmokeSensor = A9;
#endif

// Vul hier de naam in waarmee de Arduino zich aanmeldt bij MQTT, tevens het unique_id bij Home Assistant
#define CLIENT_ID  "domus_schuur"
// Vul hier de naam in waarmee de Arduino zich aanmeldt bij Home Assistant
#define DISCOVERY_ID  "Domus Schuur"
#define MODEL_ID  "Mega 2560"
#define MANUFACTURER_ID  "Arduino"
String hostname = CLIENT_ID;
// base for Home Assistant MQTT discovery (must be configured in configuration.yaml)
const String config_topic_base = "homeassistant";
// prefix for inidvidual items
const String item_prefix = "schuur";

// Vul hier het macadres in
uint8_t mac[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x09};
EthernetClient ethClient;
PubSubClient mqttClient;

// Vul hier het interval in waarmee gegevens worden verstuurd op MQTT
#define PUBLISH_DELAY 10000 // that is 10 second interval
long lastPublishTime;
// Vul hier het interval in waarmee alle statussen worden verstuurd op MQTT
#define REPORT_DELAY 60000 // that is 60 seconds interval
long lastReportTime;

// Vul hier de MQTT topic in waar deze arduino naar luistert
const char* topic_in = "domus/schuur/in";

#if defined(MQ7_present)
byte mq_state = 1;  // present state of MQ sensor: 0=preheat, 1=measure
byte mq_state_pin = 14;
byte mq_sensor_pin = A0;
byte mq_value = 0;
long mq_millis;
float co_value = 0;
const long mq_heat_interval = 60000;
const long mq_measure_interval = 90000;
const long mq_startup = 3000;
#endif

// MQTT Discovery relays (switches)
// Vul hier het aantal gebruikte relais in en de pinnen waaraan ze verbonden zijn
const byte NumberOfRelays = 1;
const byte RelayPins[] = {5};
bool RelayInitialState[] = {LOW};
String SwitchNames[] = {"Ventilator schuur"};
char* state_topic_relays = "domus/schuur/stat/relay";

// MQTT Discovery lights
// Vul hier het aantal gebruikte relais in en de pinnen waaraan ze verbonden zijn
const byte NumberOfLights = 5;
const byte LightPins[] = {13, 9, 7, 8, 44};
bool LightInitialState[] = {LOW, LOW, LOW, LOW, LOW};
bool LightBrightness[] = {false, false, false, false, true};
byte LightValue[] = {0, 0, 0, 0, 0};
String LightNames[] = {"Lamp schuur", "Lamp fietsenhok", "Lamp brandgang", "Tuinlamp achter", "Buttonleds schuur"};
const char* state_topic_lights = "domus/schuur/stat/light";
const char* cmd_topic_lights = "domus/schuur/cmd/light";

// MQTT Discovery covers
// Vul hier de gegevens in van de motorsturing voor de screens:
// 2 relais per motor: 1 x richting, 1 x motorpuls
// hiervoor gebruik ik de pulserelais en de normale relais
// de waarden zijn de indices op de onderstaande 'RelayPins' en 'PulseRelayPins' arrays
const byte NumberOfCovers = 0;
byte CoverDir[] = {2, 3}; // relay numbers for direction
byte CoverPulse[] = {2, 3}; // relay numbers for motor pulses
byte CoverState[] = {0, 0}; // 0 = open, 1 = opening, 2 = closed, 3 = closing, 4 = stopped
int CoverPos[] = {100, 100}; // position 100 = open
int CoverStart[] = {100, 100 }; // start position
byte CoverSetPos[] = {255, 255}; // set position (255 = not set)
String CoverNames[] = {"*Screen Keuken", "*Screen Huiskamer"};
long CoverDelay[] = {28000, 27000}; // time to wait for full open or close
const char* state_topic_covers = "domus/schuur/uit/screen"; // Screens (zonwering)

// MQTT Discovery locks
const byte NumberOfLocks = 0;
byte LockPulse[] = {0, 1}; // relay numbers for lock pulses (index on PulseRelayPins)
byte LockState[] = {1, 1};  // status of locks: 0 = unlocked, 1 = locked
String LockNames[] = {"*Haldeurslot", "*VoordeurSlot"};
long LockDelay[] = {2000, 250}; // pulse time for locks
const char* state_topic_locks = "domus/schuur/stat/lock"; // Locks (sloten)

// MQTT Discovery pirs (binary_sensors)
const byte NumberOfPirs = 3;
int PirSensors[] = {22, 23, 24};
int PirDebounce[] = {0, 0, 0}; // debounce time for pir or door sensor
int PreviousDetects[] = {false, false, false}; // Statusvariabele PIR sensor
byte PirState[] = {0, 0, 0};
String PirNames[] = {"PIR Fietsenhok", "Achterdeur schuur", "PIR Brandgang"};
String PirClasses[] = {"motion", "door", "motion"};
const char* state_topic_pirs = "domus/schuur/uit/pir";

// MQTT Discovery buttons (device triggers)
const int NumberOfButtons = 2;
int ButtonPins[] = {30, 31};
static byte lastButtonStates[] = {0, 0};
long lastActivityTimes[] = {0, 0};
long LongPressActive[] = {0, 0};
String ButtonNames[] = {"Knop schuur boven", "Knop schuur beneden"};
const char* state_topic_buttons = "domus/schuur/uit/button";

// MQTT Discovery sensors (sensors)
const int NumberOfSensors = 4;
String SensorNames[] = {"Temperatuur schuur", "Luchtvochtigheid schuur", "Gevoelstemperatuur schuur", "Runtime schuur"};
String SensorTypes[] = {"DHT-T", "DHT-H", "DHT-I", "TIME"};
String SensorClasses[] = {"temperature", "humidity", "temperature", "timestamp"};
String SensorUnits[] = {"°C", "%", "°C", "s"};
const char* state_topic_sensors = "domus/schuur/uit/sensor";

// Vul hier het aantal pulsrelais in
const int NumberOfPulseRelays = 0; // 0 = haldeurslot, 1 = voordeurslot, 2 = screen keuken, 3 = screen huiskamer
// Vul hier de pins in van het pulserelais.
int PulseRelayPins[] = {8, 7, 22, 24};
long PulseActivityTimes[] = {0, 0, 0, 0};
// Vul hier de default status in van het pulsrelais (sommige relais vereisen een 0, andere een 1 om te activeren)
// gebruikt 5V YwRobot relay board vereist een 0, 12 volt insteekrelais een 1, SSR relais een 1.
bool PulseRelayInitialStates[] = {HIGH, HIGH, HIGH, HIGH};
// Vul hier de pulsetijden in voor de pulserelais
long int PulseRelayTimes[] = {LockDelay[0], LockDelay[1], CoverDelay[0], CoverDelay[1]};
const char* topic_out_pulse = "domus/schuur/uit/pulse";    // Pulserelais t.b.v. deuropener

// Vul hier de uitgaande MQTT topics in
const char* topic_out = "domus/schuur/uit";

char messageBuffer[BUFFERSIZE];
char topicBuffer[BUFFERSIZE];
String ip = "";
bool startsend = HIGH;// flag for sending at startup
#if defined (DEBUG)
bool debug = true;
#else
bool debug = false;
#endif

// General variables
void ShowDebug(String tekst) {
  if (debug) {
    Serial.println(tekst);
  }
}

void(* resetFunc) (void) = 0; //declare reset function @ address 0

#define DEBOUNCE_DELAY 150
#define LONGPRESS_TIME 450

void report_state_relay()
{
  if (NumberOfRelays > 0) {
    doc.clear();
    for (int i = 0; i < NumberOfRelays; i++) {
      if (digitalRead(RelayPins[i]) == RelayInitialState[i]) {
        doc["POWER" + String(i)] = "off";
      }
      else {
        doc["POWER" + String(i)] = "on";
      }
      serializeJson(doc, messageBuffer);
    }
    ShowDebug("Sending MQTT state for relays...");
    ShowDebug(messageBuffer);
    mqttClient.publish(state_topic_relays, messageBuffer);
  }
}

void report_state_light(int index)
{
  StaticJsonDocument<256> outputdoc;
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
  else if (digitalRead(LightPins[index]) == LightInitialState[index]) {
    outputdoc["state"] = "OFF";
  }
  else {
    outputdoc["state"] = "ON";
  }
  serializeJson(outputdoc, messageBuffer);
  ShowDebug("Sending MQTT state for lights...");
  ShowDebug(messageBuffer);
  mqttClient.publish((state_topic_lights + String(index + 1)).c_str(), messageBuffer);
}

void report_state_cover()
{
  if (NumberOfCovers > 0) {
    StaticJsonDocument<256> outputdoc;
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
    ShowDebug("Sending MQTT state for covers...");
    ShowDebug(messageBuffer);
    mqttClient.publish(state_topic_covers, messageBuffer);
  }
}

void report_state_lock()
{
  if (NumberOfLocks > 0) {
    doc.clear();
    for (byte i = 0; i < NumberOfLocks ; i++) {
      if (LockState[i] == 0) {
        doc["LOCK" + String(i + 1)] = "UNLOCKED";
      }
      else {
        doc["LOCK" + String(i + 1)] = "LOCKED";
      }
    }
    serializeJson(doc, messageBuffer);
    ShowDebug("Sending MQTT state for locks...");
    ShowDebug(messageBuffer);
    mqttClient.publish(state_topic_locks, messageBuffer);
  }
}

void report_state_pir()
{
  if (NumberOfPirs > 0) {
    doc.clear();
    for (byte i = 0; i < NumberOfPirs ; i++) {
      if (PirState[i] == 0) {
        doc["PIR" + String(i + 1)] = "OFF";
      }
      else {
        doc["PIR" + String(i + 1)] = "ON";
      }
    }
    serializeJson(doc, messageBuffer);
    ShowDebug("Sending MQTT state for pirs...");
    ShowDebug(messageBuffer);
    mqttClient.publish(state_topic_pirs, messageBuffer);
  }
}

void report_state()
{
  // send data for relays
  report_state_relay();
  // send data for lights
  if (NumberOfLights > 0) {
    for (int index = 0; index < NumberOfLights ; index++) {
      report_state_light(index);
    }
  }
  // send data for covers
  report_state_cover();
  // send data for locks
  report_state_lock();
  // end send state data for MQTT discovery
}

void SetLightState(int light, String state) {
  if (state == "ON") {
    ShowDebug("Set relay " + String(LightPins[light]) + " on.");
    digitalWrite(LightPins[light], !LightInitialState[light]);
    report_state_light(light);
  }
  else if (state == "OFF") {
    ShowDebug("Set relay " + String(LightPins[light]) + " off.");
    digitalWrite(LightPins[light], LightInitialState[light]);
    LightValue[light] = 0;
    report_state_light(light);
  }
  else {
    ShowDebug("Setting pwm value " + String(LightValue[light]) + "on pin " + String(LightPins[light]));
    if (LightInitialState[light]) {
      analogWrite(LightPins[light], 255 - LightValue[light]);
    }
    else {
      analogWrite(LightPins[light], LightValue[light]);
    }
    report_state_light(light);
  }
}

void ProcessPulseRelays(int PulseRelayId) {
  // Process the timers of the pulse relays and see if we have to close them.
  if (digitalRead(PulseRelayPins[PulseRelayId]) == !PulseRelayInitialStates[PulseRelayId])
  {
    if ((millis() - PulseActivityTimes[PulseRelayId]) > PulseRelayTimes[PulseRelayId])
    {
      ShowDebug("Disabling pulse relay" + String(PulseRelayId) + ".");
      ShowDebug(String(PulseActivityTimes[PulseRelayId]));
      String messageString = "P" + String(PulseRelayId) + "0";
      messageString.toCharArray(messageBuffer, messageString.length() + 1);
      mqttClient.publish(topic_out_pulse, messageBuffer);
      digitalWrite(PulseRelayPins[PulseRelayId], PulseRelayInitialStates[PulseRelayId]);

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
          ShowDebug("Setting cover position:");
          ShowDebug(String(CoverPos[i]));
        }
      } // end update covers
      // Update lock status
      for (int i = 0 ; i < NumberOfLocks ; i++) {
        if (LockPulse[i] == PulseRelayId) {
          if (LockState[i] == 0) {   // 0 = unlocked
            LockState[i] = 1;
            report_state_lock();
          }
        }
      } // end update lock status
    }
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
  }
}

void processButtonDigital( int buttonId )
{
  int sensorReading = digitalRead( ButtonPins[buttonId] );
  if ( sensorReading == LOW ) // Input pulled low to GND. Button pressed.
  {
    if ( lastButtonStates[buttonId] == LOW )  // The button was previously un-pressed
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
      mqttClient.publish(topic_out, messageBuffer);
    }
    lastButtonStates[buttonId] = HIGH;
  }
  else {
    if (lastButtonStates[buttonId] == HIGH) {
      if (LongPressActive[buttonId]) {
        LongPressActive[buttonId] = false;
      } else {
        if ((millis() - lastActivityTimes[buttonId]) > DEBOUNCE_DELAY) // Proceed if we haven't seen a recent event on this button
        {
          ShowDebug( "Button" + String(buttonId) + " pressed" );
          String messageString = "Button" + String(buttonId);
          messageString.toCharArray(messageBuffer, messageString.length() + 1);
          mqttClient.publish(topic_out, messageBuffer);
        }
      }
      lastButtonStates[buttonId] = LOW;
    }
  }
}

#if defined(MQ7_present)
float raw_value_to_CO_ppm(float value)
{
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
    ShowDebug("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(CLIENT_ID)) {
      ShowDebug("connected");
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

void sendData() {
  float t, h, hic;
  doc.clear();
  for (int i = 0; i < NumberOfSensors; i++) {
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
      doc["sensor" + String(i + 1)] = map(analogRead(LightSensor), 0, 1023, 0, 100);
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
#if defined(MQ_present)
    else if (SensorTypes[i] == "MQ2") {
      doc["sensor" + String(i + 1)] = String(map(analogRead(SmokeSensor), 0, 1023, 0, 100));
    }
#endif
#if defined(MQ7_present)
    else if (SensorTypes[i] == "MQ7") {
      doc["sensor" + String(i + 1)] = raw_value_to_CO_ppm(co_value);
    }
#endif
  }
  serializeJson(doc, messageBuffer);
  ShowDebug("Sending MQTT state for sensors...");
  ShowDebug(messageBuffer);
  mqttClient.publish(state_topic_sensors, messageBuffer);

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
    String messageString = "P" + String(PulseRelayPort) + "1";
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out_pulse, messageBuffer);
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
    String messageString = "P" + String(PulseRelayPort) + "0";
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out_pulse, messageBuffer);
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
    String messageString = "P" + String(PulseRelayPort) + "1";
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out_pulse, messageBuffer);
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
    String messageString = "P" + String(PulseRelayPort) + "0";
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out_pulse, messageBuffer);
  }
}

void StopCover(int Cover) {
  if ((CoverState[Cover] & 1) == 1) {
    int PulseRelayPort = CoverPulse[Cover];
    // Stop pulse
    ShowDebug("Stop cover number " + String(Cover + 1));
    digitalWrite(PulseRelayPins[PulseRelayPort], PulseRelayInitialStates[PulseRelayPort]);
    String messageString = "P" + String(PulseRelayPort) + "0";
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out_pulse, messageBuffer);
  }
  if (CoverPos[Cover] == 0) {
    CoverState[Cover] = 2;
  }
  else if (CoverPos[Cover] == 100) {
    CoverState[Cover] = 0;
  }
  else {
    CoverState[Cover] = 4;
  }
}

void SetCoverPosition(int cover, int position) {
  CoverSetPos[cover] = position;
  if (CoverPos[cover] < position) {
    ShowDebug("Cover needs to open");
    CoverState[cover] = 1; // opening
    OpenCover(cover);
  }
  else {
    ShowDebug("Cover needs to close");
    CoverState[cover] = 3; // closing
    CloseCover(cover);
  }
}

void ProcessCovers(int cover) {
  if (CoverPos[cover] == CoverSetPos[cover]) {
    StopCover(cover);
  }
}

void callback(char* topic, byte * payload, byte length) {
  char msgBuffer[BUFFERSIZE];
  // I am only using one ascii character as command, so do not need to take an entire word as payload
  // However, if you want to send full word commands, uncomment the next line and use for string comparison
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
      SetLightState(index - 1, doc["state"]);
      if (LightBrightness[index - 1]) {
        LightValue[index - 1] = 255;
      }
    }
    //    SendTrigger(1); // future expansion: send device triggers
  }
  else if (strPayload[0] == '{') {
    // json message
    deserializeJson(doc, strPayload);
    for (int i = 0; i < NumberOfCovers; i++) {
      String keyword = "POSITION" + String(i + 1);
      ShowDebug("Keyword: " + String(keyword));
      if (doc.containsKey(keyword)) {
        ShowDebug("received position for cover #" + String(i + 1) + " :");
        ShowDebug(doc[keyword]);
        SetCoverPosition(i, doc[keyword]);
      }
    }
    doc.clear();
  }
  else if (strPayload[0] == 'R') {

    // Relais commando
    ShowDebug("Relay:");

    RelayPort = strPayload[1] - 48;
    if (RelayPort > 16) RelayPort -= 3;
    RelayValue = strPayload[2] - 48;

    if (RelayValue == 40) {
      ShowDebug("Relay " + String (RelayPins[RelayPort]));
      if (digitalRead(RelayPins[RelayPort]) == LOW) {
        digitalWrite(RelayPins[RelayPort], HIGH);
        ShowDebug("...to HIGH");
      }
      else {
        digitalWrite(RelayPins[RelayPort], LOW);
        ShowDebug("...to LOW");
      }
    } else {
      digitalWrite(RelayPins[RelayPort], RelayValue);
    }
    report_state_relay();
  } else if (strPayload == "IP")  {

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
  else if (strPayload == "#RESET") {
    ShowDebug("Reset command received, resetting in one second...");
    delay(1000);
    resetFunc();
  }
  else if (strPayload[0] == 'P') {  // PULSE RELAY
    int PulseRelayPort = strPayload[1] - 48;
    if (PulseRelayPort < NumberOfPulseRelays) {
      if (strPayload[2] == '1') {  // Pulserelay on
        ShowDebug("Enabling pulse relay " + String(PulseRelayPort) + ".");
        digitalWrite(PulseRelayPins[PulseRelayPort], !PulseRelayInitialStates[PulseRelayPort]);
        String messageString = "P" + String(PulseRelayPort) + "1";
        messageString.toCharArray(messageBuffer, messageString.length() + 1);
        mqttClient.publish(topic_out_pulse, messageBuffer);
        PulseActivityTimes[PulseRelayPort] = millis();
        ShowDebug("Starting pulse time on " + String(millis()));
      }
      else { // Pulserelay forced off
        ShowDebug("Disabling pulse relay " + String(PulseRelayPort) + ".");
        String messageString = "P" + String(PulseRelayPort) + "0";
        messageString.toCharArray(messageBuffer, messageString.length() + 1);
        mqttClient.publish(topic_out_pulse, messageBuffer);
        digitalWrite(PulseRelayPins[PulseRelayPort], PulseRelayInitialStates[PulseRelayPort]);
      }
    }
    else {
      ShowDebug("No such pulserelay defined!");
    }
  }
  // *** Control for screen covers ****
  // Command: O + cover number [+ pulse duration in ms]
  //  opens cover:
  //    - sets direction relay: open = inverse state
  //    - pulse duration taken from command, if not given, use parameter COVERDELAYTIME
  //    - opens motor relay for duration of pulse

  else if (strPayload[0] == 'O') {
    // Cover commando: open
    ShowDebug("Cover command : Open");
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
    ShowDebug("Cover command : Close");
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
    ShowDebug("Cover command : Stop");
    byte CoverPort = strPayload[1] - 48;
    ShowDebug("Cover number " + String(CoverPort));
    if (CoverPort <= NumberOfCovers) {
      StopCover(CoverPort - 1);
    }
    else {
      ShowDebug("No such cover defined: " + String(CoverPort));
    }
  }
  // MQTT Discovery Lock commands
  else if (strPayload[0] == 'L') {
    // Lock commando: lock
    ShowDebug("Lock command : lock");
    byte LockPort = strPayload[1] - 48;
    ShowDebug("Lock number " + String(LockPort));
    int PulseRelayPort = LockPulse[LockPort - 1];
    if (LockPort <= NumberOfLocks) {
      // End opening pulse
      digitalWrite(PulseRelayPins[PulseRelayPort], PulseRelayInitialStates[PulseRelayPort]);
      String messageString = "P" + String(PulseRelayPort) + "0";
      messageString.toCharArray(messageBuffer, messageString.length() + 1);
      mqttClient.publish(topic_out_pulse, messageBuffer);
      LockState[LockPort - 1] = 1;
      report_state_lock();
    }
    else {
      ShowDebug("No such lock defined: " + String(LockPort));
    }
  }
  else if (strPayload[0] == 'U') {
    // Lock commando: unlock
    ShowDebug("Lock command : unlock");
    byte LockPort = strPayload[1] - 48;
    ShowDebug("Lock number " + String(LockPort));
    int PulseRelayPort = LockPulse[LockPort - 1];
    if (LockPort <= NumberOfLocks) {
      // Set opening pulse
      digitalWrite(PulseRelayPins[PulseRelayPort], !PulseRelayInitialStates[PulseRelayPort]);
      String messageString = "P" + String(PulseRelayPort) + "1";
      messageString.toCharArray(messageBuffer, messageString.length() + 1);
      mqttClient.publish(topic_out_pulse, messageBuffer);
      PulseActivityTimes[PulseRelayPort] = millis();
      LockState[LockPort - 1] = 0;
      report_state_lock();
    }
    else {
      ShowDebug("No such lock defined: " + String(LockPort));
    }
  }
  else {
    // Onbekend commando
    ShowDebug("Unknown value");
    mqttClient.publish(topic_out, "Unknown command");
  }
}

void check_pir(byte pirid)
{
  // ...read out the PIR sensors...
  if (digitalRead(PirSensors[pirid]) == HIGH) {
    if (!PreviousDetects[pirid]) {
      ShowDebug("Pir " + String(pirid) + " on.");
      PreviousDetects[pirid] = true;
      PirState[pirid] = 1;
      report_state_pir();
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

void setDeviceInfo(char* configtopic) {
  JsonObject device = doc.createNestedObject("dev");
  JsonArray identifiers = device.createNestedArray("ids");
  identifiers.add(CLIENT_ID);
  JsonArray connections = device.createNestedArray("cns");
  connections.add(serialized("[\"ip\",\"" + String(ip) + "\"]"));
  connections.add(serialized("[\"mac\",\"" + mac2String(mac) + "\"]"));
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

//void SendTrigger(int Button) {
//  doc.clear();
//  doc["automation_type"] = "trigger";
//  doc["topic"] = state_topic_buttons;
//  doc["type"] = "button_short_press";
//  doc["subtype"] = "button_1";
//  setDeviceInfo((config_topic_base + "/device_automation/" + ButtonNames[Button] + "/config").c_str());
//}

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
    doc["pos_t"] = state_topic_covers;
    doc["cmd_t"] = topic_in;
    doc["pl_open"] = "O" + String(i + 1);
    doc["pl_cls"] = "C" + String(i + 1);
    doc["pl_stop"] = "S" + String(i + 1);
    doc["position_open"] = 100;
    doc["position_closed"] = 0;
    doc["set_pos_t"] = topic_in;
    doc["set_pos_tpl"] = "{ \"POSITION" + String(i + 1) +  "\": {{ position }} }";
    doc["val_tpl"] = "{{value_json.POSITION" + String(i + 1) + "}}";
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
  //  for (int i = 0; i < NumberOfButtons ; i++ ) {
  //    doc.clear();
  //    doc["automation_type"] = "trigger";
  //    doc["topic"] = state_topic_buttons;
  //    doc["type"] = "button_short_press";
  //    doc["subtype"] = "button_1";
  //    setDeviceInfo((config_topic_base + "/device_automation/" + ButtonNames[i] + "/config").c_str());
  //  }
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
    doc["unit_of_meas"] = SensorUnits[i];
    doc["val_tpl"] = " {{value_json.sensor" + String(i + 1) + "}}";
    setDeviceInfo((config_topic_base + "/sensor/" + item_prefix + "_sensor"  + String(i + 1) + "/config").c_str());
  }
  //  end send config data for MQTT discovery
}

void setup() {

  if (debug) {
    Serial.begin(115200);
    ShowDebug(CLIENT_ID);
    ShowDebug(String(MQTT_MAX_PACKET_SIZE));
  }

  for (byte thisPin = 0; thisPin < NumberOfRelays; thisPin++) {
    ShowDebug("Relay: " + String(RelayPins[thisPin]));
    pinMode(RelayPins[thisPin], OUTPUT);
    digitalWrite(RelayPins[thisPin], RelayInitialState[thisPin]);
  }

  for (byte thisPin = 0; thisPin < NumberOfLights; thisPin++) {
    ShowDebug("Light: " + String(LightPins[thisPin]));
    pinMode(LightPins[thisPin], OUTPUT);
    digitalWrite(LightPins[thisPin], LightInitialState[thisPin]);
  }

  for (int thisPin = 0; thisPin < NumberOfPulseRelays; thisPin++) {
    pinMode(PulseRelayPins[thisPin], OUTPUT);
    ShowDebug("Pulse relay: " + String(PulseRelayPins[thisPin]));
    digitalWrite(PulseRelayPins[thisPin], PulseRelayInitialStates[thisPin]);
  }

  for (int thisButton = 0; thisButton < NumberOfButtons; thisButton++) {
    ShowDebug("Button: " + String(ButtonPins[thisButton]));
    pinMode(ButtonPins[thisButton], INPUT_PULLUP);
  }

#if defined(DHT_present)
  dht.begin();
  ShowDebug("DHT-22 sensor: 3");
#endif

#if defined(LDR_present)
  pinMode(LightSensor, INPUT);
  ShowDebug("LDR sensor: A10");
#endif

#if defined(DS18B20_present)
  ShowDebug("DS18B20 sensor: 16");
#endif

#if defined(BMP_present)
  if (!bmp.begin()) {
    ShowDebug("Could not find a valid BMP280 sensor, check wiring!");
  }
  else {
    ShowDebug("BMP280 sensor (i2c): 20 (SDA), 21 (SCL)");
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

  for (byte pirid = 0; pirid < NumberOfPirs; pirid++) {
    ShowDebug("Pir: " + String(PirSensors[pirid]));
    pinMode(PirSensors[pirid], INPUT_PULLUP);
  }

  ShowDebug("Network...");
  // attempt to connect to network:
  //   setup ethernet communication using DHCP
  if (Ethernet.begin(mac) == 0) {

    ShowDebug(F("No DHCP"));
    delay(1000);
    resetFunc();
  }
  ShowDebug(F("Ethernet via DHCP"));
  ShowDebug("IP address: ");
  ip = String (Ethernet.localIP()[0]);
  ip = ip + ".";
  ip = ip + String (Ethernet.localIP()[1]);
  ip = ip + ".";
  ip = ip + String (Ethernet.localIP()[2]);
  ip = ip + ".";
  ip = ip + String (Ethernet.localIP()[3]);
  ShowDebug(ip);
  ShowDebug("");

  // setup mqtt client
  mqttClient.setClient(ethClient);
  mqttClient.setServer(MQTTSERVER, 1883); // or local broker
  ShowDebug("MQTT set up");
  mqttClient.setCallback(callback);
  ShowDebug("Ready to send data");
  lastPublishTime = millis();
}

void loop() {
  // Main loop, where we check if we're connected to MQTT...
  if (!mqttClient.connected()) {
    ShowDebug("Not Connected!");
    reconnect();
  }

  // ... then send all relay stats and discovery info when we've just started up....
  if (startsend) {
    reportMQTTdisco();
    report_state();
    startsend = false;
  }

  // ...handle the PulseRelays, ...
  for (int id = 0; id < NumberOfPulseRelays; id++) {
    ProcessPulseRelays(id);
  }

  // handle the cover position
  for (int id = 0; id < NumberOfCovers; id++) {
    ProcessCovers(id);
  }

  // ...read out the PIR sensors...
  for (int id = 0; id < NumberOfPirs; id++) {
    check_pir(id);
  }

#if defined(MQ7_present)
  // ...process the MQ-7 sensor...
  if (mq_state == 0) {
    digitalWrite(mq_state_pin, HIGH);
    if ( mq_millis < millis() ) {
      ShowDebug("MQ-7 measure...");
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
      ShowDebug("MQ-7 heatup...");
      mq_state = 0;
      mq_millis = millis() + mq_heat_interval;
    }
  }
#endif

  // ...see if it's time to send new data, ....
  if (millis() - lastPublishTime > PUBLISH_DELAY) {
    lastPublishTime = millis();
    sendData();
  }
  else if (millis() - lastReportTime > REPORT_DELAY) {
    lastReportTime = millis();
    report_state();
  }
  else {
    for (int id = 0; id < NumberOfButtons; id++) {
      processButtonDigital(id);
    }
  }

  // and loop.
  mqttClient.loop();
}
