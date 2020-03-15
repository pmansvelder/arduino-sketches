/*
          <========Arduino Sketch for Arduino Mega =========>
          Locatie: Meterkast
          Macadres: 00:01:02:03:04:0B
          Aansluiting via UTP kabel:
          - 5V : bruin
          - GND: wit

          Pins used:
          0: Serial
          1: Serial
          2: PWM voor LEDs
          3: DHT-22 sensor
          4: <in gebruik voor W5100>
          5: Relay 0 (not connected)
          6: Relay 1 (not connected)
          7: PulseRelay 1 (Pulse, Voordeuropener)
          8: PulseRelay 0 (Pulse, Haldeuropener)
          9: Button 2 (keuken)
          10: <in gebruik voor W5100>
          11: Button 0 (huiskamer)
          12: Button 1 (huiskamer)
          19: PIR Hal
          20: SDA
          21: SCL
          22: Relay screen 1
          23: Pulserelay screen 1
          24: Relay screen 2
          25: Pulserelay screen 2
          28: PIR Keuken
          29: Magneetcontact voordeur
          30: Relay 3: SSR Relais voor keukenlamp
          31: Relay 4: SSR Relais voor plafondlamp huiskamer

          50: <in gebruik voor W5100>
          51: <in gebruik voor W5100>
          52: <in gebruik voor W5100>
          53: <in gebruik voor W5100>

          A0:
          A1:
          A2:
          A3:
          A4:
          A5:

          incoming topic: domus/mk/in

          Arduino Mega with W5100 ethernet shield used as MQTT client
          It will connect over Ethernet to the MQTT broker and controls digital outputs (LED, relays)
          The topics have the format "domus/hobby/uit" for outgoing messages and
          "domus/mk/in" for incoming messages.

          The outgoing topics are

          domus/mk/uit        // Relaisuitgangen: R<relaisnummer><status>

          Here, each relay state is reported using the same syntax as the switch command:
          R<relay number><state>

          There is only one incoming topic:
          domus/mk/in
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


          Adapted 24-02-2018 by Peter Mansvelder:
          - added sensors for light and smoke (MQ-2), reporting on output topics
          - added alarm function for smoke with buzzer
          - added pulse relay

          Adapted 22-07-2018 by Peter Mansvelder
          - added smart meter P1 input (later removed)

          Adapted 4-11-2018 by Peter Mansvelder
          - added cover control

          Adapted 15-3-2020 by Peter Mansvelder
          - set up for MQTT discovery

*/

// Vul hier de naam in waarmee de Arduino zich aanmeldt bij MQTT, tevens het unique_id bij Home Assistant
#define CLIENT_ID  "domus_meterkast_screens"
// Vul hier de naam in waarmee de Arduino zich aanmeldt bij Home Assistant
#define DISCOVERY_ID  "Domus Mega Meterkast"
#define MODEL_ID  "Mega 2560"
#define MANUFACTURER_ID  "Arduino"

// base for Home Assistant MQTT discovery (must be configured in configuration.yaml)
const String config_topic_base = "homeassistant";
// prefix for inidvidual items
const String item_prefix = "meterkast";

#include "secrets.h"

// parameters to tune memory use
//#define BMP280 1 // use BMP280 sensor
//#define DHT_present 0 // use DHT sensor
//#define MQ_present 0 // MQ-x gas sensor
#define DEBUG 1 // Zet debug mode aan

#include <Ethernet.h>           // Ethernet.h library
#include "PubSubClient.h"       //PubSubClient.h Library from Knolleary, must be adapted: #define MQTT_MAX_PACKET_SIZE 512

//#include <Adafruit_Sensor.h>
//#include <Adafruit_BMP280.h>    // Adafruit BMP280 library

#include "ArduinoJson.h"
StaticJsonDocument<256> doc;

#define BUFFERSIZE 512          // default 100

#if defined(DHT_present)
#include <DHT.h>
#define DHT_PIN 3 // Vul hier de pin in van de DHT11 sensor
DHT dht(DHT_PIN, DHT22);
#endif

// Vul hier het interval in waarmee gegevens worden verstuurd op MQTT
#define PUBLISH_DELAY 60000 // that is 60 seconds interval

String hostname = CLIENT_ID;

// Vul hier de MQTT topic in waar deze arduino naar luistert
const char* topic_in = "domus/mk/in";

#if defined(DHT_present)
const char* topic_out_temp = "domus/mk/uit/temp";
const char* topic_out_hum = "domus/mk/uit/vocht";
const char* topic_out_heat = "domus/mk/uit/warmte";
#endif

#if defined(MQ_present)
const char* topic_out_gas = "domus/mk/uit/gas";
byte mq_state = 1;  // present state of MQ sensor: 0=preheat, 1=measure
byte mq_state_pin = 5;
byte mq_sensor_pin = A0;
byte mq_value = 0;
long mq_millis;
float co_value = 0;
const long mq_heat_interval = 60000;
const long mq_measure_interval = 90000;
const long mq_startup = 3000;
#endif

// MQTT Discovery relays
// Vul hier het aantal gebruikte relais in en de pinnen waaraan ze verbonden zijn
const byte NumberOfRelays = 4;
const byte RelayPins[NumberOfRelays] = {5, 6, 22, 24};
bool RelayInitialState[NumberOfRelays] = {HIGH, HIGH, HIGH, HIGH};
String SwitchNames[NumberOfRelays] = {"Mediaplayer Keuken", "CV-ketel", "Screen keuken", "Screen Huiskamer"};
char* state_topic_relays = "domus/mk/stat/relay";

// MQTT Discovery lights
// Vul hier het aantal gebruikte relais in en de pinnen waaraan ze verbonden zijn
const byte NumberOfLights = 3;
const byte LightPins[NumberOfLights] = {30, 31, 2};
bool LightInitialState[NumberOfLights] = {HIGH, HIGH, HIGH};
bool LightBrightness[NumberOfLights] = {false, false, true};
byte LightValue[NumberOfLights] = {0, 0, 0};
String LightNames[NumberOfLights] = {"Keuken", "Plafondlamp", "Buttonleds"};
const char* state_topic_lights = "domus/mk/stat/light";
const char* cmd_topic_lights = "domus/mk/cmd/light";

// MQTT Discovery covers
// Vul hier de gegevens in van de motorsturing voor de screens:
// 2 relais per motor: 1 x richting, 1 x motorpuls
// hiervoor gebruik ik de pulserelais en de normale relais
// de waarden zijn de indices op de onderstaande 'RelayPins' en 'PulseRelayPins' arrays
const byte NumberOfCovers = 2;
byte CoverDir[NumberOfCovers] = {2, 3}; // relay numbers for direction
byte CoverPulse[NumberOfCovers] = {2, 3}; // relay numbers for motor pulses
byte CoverState[NumberOfCovers] = {0, 0}; // 0 = open, 1 = opening, 2 = closed, 3 = closing, 4 = stopped
int CoverPos[NumberOfCovers] = {100, 100}; // position 100 = open
int CoverStart[NumberOfCovers] = {100, 100 }; // start position
byte CoverSetPos[NumberOfCovers] = {255, 255}; // set position (255 = not set)
String CoverNames[NumberOfCovers] = {"Screen Keuken", "Screen Huiskamer"};
#define COVERDELAYTIME 30000 // time to wait for full open or close
const char* state_topic_covers = "domus/mk/uit/screen"; // Screens (zonwering)

// MQTT Discovery locks
const byte NumberOfLocks = 2;
byte LockPulse[NumberOfLocks] = {0, 1}; // relay numbers for lock pulses (index on PulseRelayPins)
byte LockState[NumberOfLocks] = {1, 1};  // status of locks: 0 = unlocked, 1 = locked
String LockNames[NumberOfLocks] = {"Haldeurslot", "VoordeurSlot"};
#define LOCKPULSETIME 3000 // pulse time for locks
const char* state_topic_locks = "domus/mk/stat/lock"; // Locks (sloten)

// MQTT Discovery pirs (binary_sensors)
const byte NumberOfPirs = 3;
int PirSensors[NumberOfPirs] = {28, 29, 19};
int PirDebounce[NumberOfPirs] = {0, 0, 0}; // debounce time for pir or door sensor
int PreviousDetects[NumberOfPirs] = {false, false, false}; // Statusvariabele PIR sensor
byte PirState[NumberOfPirs] = {0, 0, 0};
String PirNames[NumberOfPirs] = {"PIR Hal", "PIR Keuken", "Voordeur"};
String PirClasses[NumberOfPirs] = {"motion", "motion", "door"};
const char* state_topic_pirs = "domus/mk/uit/pir";

// MQTT Discovery buttons (device triggers)
const int NumberOfButtons = 3;
int ButtonPins[] = {11, 12, 9};
static byte lastButtonStates[] = {0, 0, 0};
long lastActivityTimes[] = {0, 0, 0};
long LongPressActive[] = {0, 0, 0};
String ButtonNames[NumberOfButtons] = {"Knop Keuken", "Keuken", "Voordeur"};
const char* state_topic_buttons = "domus/mk/uit/button";

// Vul hier het aantal pulsrelais in
const int NumberOfPulseRelays = 4; // 0 = haldeur, 1 = voordeur, 2 = screen keuken, 3 = screen huiskamer
// Vul hier de pins in van het pulserelais.
int PulseRelayPins[NumberOfPulseRelays] = {7, 8, 23, 25};
long PulseActivityTimes[NumberOfPulseRelays] = {0, 0, 0, 0};
// Vul hier de default status in van het pulsrelais (sommige relais vereisen een 0, andere een 1 om te activeren)
// gebruikt 5V YwRobot relay board vereist een 0, 12 volt insteekrelais een 1, SSR relais een 1.
bool PulseRelayInitialStates[NumberOfPulseRelays] = {HIGH, HIGH, HIGH, HIGH};
// Vul hier de pulsetijden in voor de pulserelais
long int PulseRelayTimes[NumberOfPulseRelays] = {LOCKPULSETIME, LOCKPULSETIME, COVERDELAYTIME, COVERDELAYTIME};
const char* topic_out_pulse = "domus/mk/uit/pulse";    // Pulserelais t.b.v. deuropener

// Vul hier de uitgaande MQTT topics in
const char* topic_out = "domus/mk/uit";

char messageBuffer[BUFFERSIZE];
char topicBuffer[BUFFERSIZE];
String ip = "";
bool startsend = HIGH;// flag for sending at startup
#if defined (DEBUG)
bool debug = true;
#else
bool debug = false;
#endif

// Vul hier het macadres in
uint8_t mac[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x0B};
EthernetClient ethClient;
PubSubClient mqttClient;

long previousMillis;

// General variables
void ShowDebug(String tekst) {
  if (debug) {
    Serial.println(tekst);
  }
}

void(* resetFunc) (void) = 0; //declare reset function @ address 0

#define DEBOUNCE_DELAY 150
#define LONGPRESS_TIME 450

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
      ShowDebug("failed, rc=" + String(mqttClient.state()));
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

#if defined(DHT_present)
  float h = CheckIsNan(dht.readHumidity(), 0);
  float t = CheckIsNan(dht.readTemperature(), 0);
  float hic = CheckIsNan(dht.computeHeatIndex(t, h, false), -99);

  //  Send Temperature sensor
  ShowDebug("T: " + String(t));
  sendMessage(String(t), topic_out_temp);

  //  Send Humidity sensor
  ShowDebug("H: " + String(h));
  sendMessage(String(h), topic_out_hum);

  //  Send Heat index sensor
  ShowDebug("HI: " + String(hic));
  sendMessage(String(hic), topic_out_heat);
#endif

#if defined(BMP280)
  if (bmp_present) {
    float t = bmp.readTemperature();
    sendMessage(String(t), topic_out_bmptemp);
    long p = bmp.readPressure();
    sendMessage(String(p), topic_out_pressure);
  }
#endif

#if defined(MQ_present)
  ShowDebug("CO: " + String(raw_value_to_CO_ppm(co_value)));
  sendMessage(String(raw_value_to_CO_ppm(co_value)), topic_out_gas);
  ShowDebug(String(millis()));
  ShowDebug(String(mq_millis));
  ShowDebug(String(mq_state));
#endif
  report_state();
}

void report_state()
{
  // send data for relays
  report_state_relay();
  // send data for lights
  for (int index = 0; index < NumberOfLights ; index++) {
    report_state_light(index);
  }
  // send data for covers
  report_state_cover();
  // send data for locks
  report_state_lock();
  // end send state data for MQTT discovery
}

void report_state_relay()
{
  doc.clear();
  for (int i = 0; i < NumberOfRelays; i++) {
    if (digitalRead(RelayPins[i]) == HIGH) {
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

void report_state_lock()
{
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

void report_state_pir()
{
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

void OpenCover(int Cover) {
  int PulseRelayPort = CoverPulse[Cover];
  ShowDebug("Pulse relay index = " + String(PulseRelayPort));
  ShowDebug("Pulse relay port = " + String(PulseRelayPins[PulseRelayPort]));
  ShowDebug("Status port = " + String(digitalRead(PulseRelayPins[PulseRelayPort])));
  // Check if another command is already running
  if (digitalRead(PulseRelayPins[PulseRelayPort]) == PulseRelayInitialStates[PulseRelayPort]) {
    ShowDebug("Opening cover number " + String(Cover + 1));
    //    PulseRelayTimes[PulseRelayPort] = COVERDELAYTIME;
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
    //    PulseRelayTimes[PulseRelayPort] = COVERDELAYTIME;
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
    //    doc["name"] = "JSON switch" + String(i + 1);
    doc["name"] = SwitchNames[i];
    doc["uniq_id"] = item_prefix + "_switch" + String(i + 1);
    doc["stat_t"] = state_topic_relays;
    doc["cmd_t"] = topic_in;
    doc["pl_on"] = "R" + String(i) + "0";
    doc["pl_off"] = "R" + String(i) + "1";
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
    //    doc["stat_t"] = state_topic_covers;
    doc["pos_t"] = state_topic_covers;
    doc["cmd_t"] = topic_in;
    doc["pl_open"] = "O" + String(i + 1);
    doc["pl_cls"] = "C" + String(i + 1);
    doc["pl_stop"] = "S" + String(i + 1);
    doc["position_open"] = 100;
    doc["position_closed"] = 0;
    doc["set_pos_t"] = topic_in;
    doc["set_pos_tpl"] = "{ \"POSITION" + String(i + 1) +  "\": {{ position }} }";
    //    doc["stat_open"] = "open";
    //    doc["stat_clsd"] = "closed";
    //    doc["state_opening"] = "opening";
    //    doc["state_closing"] = "closing";
    //    doc["val_tpl"] = " {{value_json.COVER" + String(i + 1) + "}}";
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
    ShowDebug("Enabling pulse relay pin " + String(PulseRelayPins[thisPin]));
    digitalWrite(PulseRelayPins[thisPin], PulseRelayInitialStates[thisPin]);
  }

  for (int thisButton = 0; thisButton < NumberOfButtons; thisButton++) {
    ShowDebug("Button: " + String(ButtonPins[thisButton]));
    pinMode(ButtonPins[thisButton], INPUT_PULLUP);
  }

#if defined(DHT_present)
  dht.begin();
#endif

  //  pinMode(SmokeSensor, INPUT);
  //  pinMode(PWMoutput, OUTPUT);
  //  pinMode(LightSensor, INPUT);

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
  previousMillis = millis();

#if defined(MQ_present)
  mq_millis = millis();
  ShowDebug("Pin " + String(mq_sensor_pin) + " is MQ sensor");
#endif
}

void loop() {
  // Main loop, where we check if we're connected to MQTT...
  if (!mqttClient.connected()) {
    ShowDebug("MQTT not Connected!");
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

#if defined(MQ_present)
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
  if (millis() - previousMillis > PUBLISH_DELAY) {
    previousMillis = millis();
    sendData();
  }
  else {
    for (int id = 0; id < NumberOfButtons; id++) {
      processButtonDigital(id);
    }
  }

  // and loop.
  mqttClient.loop();
}
