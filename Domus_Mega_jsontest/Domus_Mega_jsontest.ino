/*
          <========Arduino Sketch for Arduino Mega =========>
          Locatie: Test

          Aansluiting via UTP kabel:
          - 5V : bruin
          - GND: wit

          Pins used:
          0: Serial
          1: Serial
          2: PIR Sensor                         - UTP kabel : groen
          3: DHT-22 sensor                      - UTP kabel : wit
          4: <in gebruik voor W5100>
          5: LED 1
          6: LED 2
          7: LED 3
          8: LED 4
          9:
          30:
          32:
          34:
          36:

          10: <in gebruik voor W5100>
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

          incoming topic: domus/test/in

          Arduino Mega with W5100 ethernet shield used as MQTT client
          It will connect over Ethernet to the MQTT broker and controls digital outputs (LED, relays)
          The topics have the format "domus/hobby/uit" for outgoing messages and
          "domus/test/in" for incoming messages.

          The outgoing topics are

          domus/test/uit        // Relaisuitgangen: R<relaisnummer><status>

          Here, each relay state is reported using the same syntax as the switch command:
          R<relay number><state>

          There is only one incoming topic:
          domus/test/in
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

*/

// Vul hier de naam in waarmee de Arduino zich aanmeldt bij MQTT, tevens het unique_id bij Home Assistant
#define CLIENT_ID  "domus_test_json"
// Vul hier de naam in waarmee de Arduino zich aanmeldt bij Home Assistant
#define DISCOVERY_ID  "Domus Mega Test"
#define MODEL_ID  "Mega 2560"
#define MANUFACTURER_ID  "Arduino"

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

// Vul hier het interval in waarmee switch en sensorgegevens worden verstuurd op MQTT
#define PUBLISH_DELAY 1500 // that is 1.5 seconds interval

String hostname = CLIENT_ID;

// Vul hier de data in van de PIRs
byte NumberOfPirs = 0;
int PirSensors[] = {};
int PreviousDetects[] = {}; // Statusvariabele PIR sensor

// Vul hier de MQTT topic in waar deze arduino naar luistert
const char* topic_in = "domus/test/in";

#if defined(DHT_present)
const char* topic_out_temp = "domus/test/uit/temp";
const char* topic_out_hum = "domus/test/uit/vocht";
const char* topic_out_heat = "domus/test/uit/warmte";
#endif

const char* topic_out_pir = "domus/test/uit/pir";

#if defined(MQ_present)
const char* topic_out_gas = "domus/test/uit/gas";
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

// Vul hier het aantal gebruikte relais in en de pinnen waaraan ze verbonden zijn
const byte NumberOfRelays = 4;
const byte RelayPins[] = {5, 6, 7, 8};
bool RelayInitialState[] = {HIGH, HIGH, HIGH, HIGH};

// Vul hier de gegevens in van de motorsturing voor de screens:
// 2 relais per motor: 1 x richting, 1 x motorpuls
// hiervoor gebruik ik de pulserelais en de normale relais
// de waarden zijn de indices op de onderstaande 'RelayPins' en 'PulseRelayPins' arrays
const byte NumberOfCovers = 2;
byte CoverDir[NumberOfCovers] = {1, 3}; // relay numbers for direction
byte CoverPulse[NumberOfCovers] = {0, 1}; // relay numbers for motor pulses
byte CoverState[NumberOfCovers] = {0, 0}; // 0 = open, 1 = opening, 2 = closed, 3 = closing
byte CoverPos[NumberOfCovers] = {100, 100}; // position 100 = open
#define COVERDELAYTIME 30000 // time to wait for full open or close

// Vul hier het aantal pulsrelais in
const int NumberOfPulseRelays = 2; // 0 = haldeur, 1 = voordeur, 2 = screen keuken, 3 = screen huiskamer
// Vul hier de pins in van het pulserelais.
int PulseRelayPins[] = {5, 7};
long PulseActivityTimes[] = {0, 0};
// Vul hier de default status in van het pulsrelais (sommige relais vereisen een 0, andere een 1 om te activeren)
// gebruikt 5V YwRobot relay board vereist een 0, 12 volt insteekrelais een 1, SSR relais een 1.
bool PulseRelayInitialStates[] = {HIGH, HIGH};
// Vul hier de pulsetijden in voor de pulserelais
long int PulseRelayTimes[] = {COVERDELAYTIME, COVERDELAYTIME};

// Vul hier de uitgaande MQTT topics in
const char* topic_out = "domus/test/uit";
const char* topic_relays = "homeassistant/switch/jsontest";
const char* state_topic_relays = "homeassistant/switch/jsontest/state";
const char* topic_out_pulse = "domus/test/uit/pulse";    // Pulserelais t.b.v. deuropener
const char* topic_out_screen = "domus/test/uit/screen"; // Screens (zonwering)

//char* config_topic_relays = "homeassistant/switch/jsontest/config";

const String config_topic_base = "homeassistant";

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
uint8_t mac[6] = {0xF0, 0x01, 0x02, 0x03, 0x04, 0x4B};
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

// Vul hier het aantal knoppen in en de pinnen waaraan ze verbonden zijn
int NumberOfButtons = 0;
//int ButtonPins[] = {6, 7, 8, 9};
//static byte lastButtonStates[] = {0, 0, 0, 0};
//long lastActivityTimes[] = {0, 0, 0, 0};
//long LongPressActive[] = {0, 0, 0, 0};

#define DEBOUNCE_DELAY 150
#define LONGPRESS_TIME 450

void ProcessPulseRelays(int PulseRelayId) {
  // Process the timers of the pulse relays and see if we have to close them.
  if (digitalRead(PulseRelayPins[PulseRelayId]) == !PulseRelayInitialStates[PulseRelayId]) {
    if ((millis() - PulseActivityTimes[PulseRelayId]) > PulseRelayTimes[PulseRelayId])
    {
      ShowDebug("Disabling pulse relay" + String(PulseRelayId) + ".");
      ShowDebug(String(PulseActivityTimes[PulseRelayId]));
      String messageString = "P" + String(PulseRelayId) + "0";
      messageString.toCharArray(messageBuffer, messageString.length() + 1);
      mqttClient.publish(topic_out_pulse, messageBuffer);
      digitalWrite(PulseRelayPins[PulseRelayId], PulseRelayInitialStates[PulseRelayId]);
      for (int i = 0 ; i < NumberOfCovers ; i++) {
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
      }
    }
    else {
      for (int i = 0 ; i < NumberOfCovers ; i++) {
        if (CoverPulse[i] == PulseRelayId) {
          CoverPos[i] = int(100 * (millis() - PulseActivityTimes[PulseRelayId]) / PulseRelayTimes[PulseRelayId]);
          if (CoverState[i] == 3) {     // 3 = closing
            CoverPos[i] = 100 - CoverPos[i] ;
          }
        }
      }
    }
  }
}

//void processButtonDigital( int buttonId )
//{
//  int sensorReading = digitalRead( ButtonPins[buttonId] );
//  if ( sensorReading == LOW ) // Input pulled low to GND. Button pressed.
//  {
//    if ( lastButtonStates[buttonId] == LOW )  // The button was previously un-pressed
//    {
//      if ((millis() - lastActivityTimes[buttonId]) > DEBOUNCE_DELAY) // Proceed if we haven't seen a recent event on this button
//      {
//        lastActivityTimes[buttonId] = millis();
//      }
//    }
//    else if ((millis() - lastActivityTimes[buttonId] > LONGPRESS_TIME) && (!LongPressActive[buttonId]))// Button long press
//    {
//      LongPressActive[buttonId] = true;
//      ShowDebug( "Button" + String(buttonId) + " long pressed" );
//      String messageString = "Button" + String(buttonId) + "_long";
//      messageString.toCharArray(messageBuffer, messageString.length() + 1);
//      mqttClient.publish(topic_out, messageBuffer);
//    }
//    lastButtonStates[buttonId] = HIGH;
//  }
//  else {
//    if (lastButtonStates[buttonId] == HIGH) {
//      if (LongPressActive[buttonId]) {
//        LongPressActive[buttonId] = false;
//      } else {
//        if ((millis() - lastActivityTimes[buttonId]) > DEBOUNCE_DELAY) // Proceed if we haven't seen a recent event on this button
//        {
//          ShowDebug( "Button" + String(buttonId) + " pressed" );
//          String messageString = "Button" + String(buttonId);
//          messageString.toCharArray(messageBuffer, messageString.length() + 1);
//          mqttClient.publish(topic_out, messageBuffer);
//        }
//      }
//      lastButtonStates[buttonId] = LOW;
//    }
//  }
//}

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
  // send state data for MQTT discovery
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
  ShowDebug(topic_relays);
  mqttClient.publish(state_topic_relays, messageBuffer);
  // send data for covers
  report_state_cover(1);
  // end send state data for MQTT discovery
}

void report_state_cover(int CoverPort)
{
  doc.clear();
  for (byte i = 0; i < NumberOfCovers ; i++) {
    if (CoverState[i] == 0) {
      doc["COVER" + String(i + 1)] = "open";
    }
    else if (CoverState[i] == 1) {
      doc["COVER" + String(i + 1)] = "opening";
    }
    else if (CoverState[i] == 2) {
      doc["COVER" + String(i + 1)] = "closed";
    }
    else if (CoverState[i] == 3) {
      doc["COVER" + String(i + 1)] = "closing";
    }
    doc["POSITION" + String(i + 1)] = CoverPos[i];
  }
  serializeJson(doc, messageBuffer);
  ShowDebug("Sending MQTT state for covers...");
  ShowDebug(messageBuffer);
  ShowDebug(topic_out_screen);
  mqttClient.publish(topic_out_screen, messageBuffer);
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
  ShowDebug(strPayload);
  ShowDebug("Message arrived");
  ShowDebug(topic);
  ShowDebug(strPayload);

  byte RelayPort;
  byte RelayValue;

  if (strPayload[0] == 'R') {

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
    report_state();
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
    report_state();
  }

  else if (strPayload == "AOF") {
    // Alle relais uit
    for (byte i = 0 ; i < NumberOfRelays; i++) {
      digitalWrite(RelayPins[i], RelayInitialState[i]);
    }
    report_state();
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
  //  example command: O120000 = open cover 1, use 20000ms (=20s) pulse
  //
  else if (strPayload[0] == 'O') {
    // Cover commando: open
    ShowDebug("Cover command : Open");
    byte CoverPort = strPayload[1] - 48;
    // 2nd char is # of cover
    ShowDebug("Cover number " + String(CoverPort));
    if (CoverPort <= NumberOfCovers) {
      int PulseRelayPort = CoverPulse[CoverPort - 1];
      ShowDebug("Pulse relay index = " + String(PulseRelayPort));
      ShowDebug("Pulse relay port = " + String(PulseRelayPins[PulseRelayPort]));
      ShowDebug("Status port = " + String(digitalRead(PulseRelayPins[PulseRelayPort])));
      // Check if another command is already running
      if (digitalRead(PulseRelayPins[PulseRelayPort]) == PulseRelayInitialStates[PulseRelayPort]) {
        ShowDebug("Opening cover number " + String(CoverPort));
        // allow for custom pulse time
        long PulseTime = strPayload.substring(2).toInt();
        if (PulseTime == 0) PulseTime = COVERDELAYTIME;
        PulseRelayTimes[PulseRelayPort] = PulseTime;
        // Set direction relay
        digitalWrite(RelayPins[CoverDir[CoverPort - 1]], !RelayInitialState[CoverDir[CoverPort - 1]]);
        // Set opening pulse
        digitalWrite(PulseRelayPins[PulseRelayPort], !PulseRelayInitialStates[PulseRelayPort]);
        String messageString = "P" + String(PulseRelayPort) + "1";
        messageString.toCharArray(messageBuffer, messageString.length() + 1);
        mqttClient.publish(topic_out_pulse, messageBuffer);
        PulseActivityTimes[PulseRelayPort] = millis();
        // report state of screen
        CoverState[CoverPort - 1] = 1; // 1 is opening
        report_state_cover(CoverPort);
        //        messageString = "O" + String(CoverPort);
        //        messageString.toCharArray(messageBuffer, messageString.length() + 1);
        //        mqttClient.publish(topic_out_screen, messageBuffer);
      }
      else {
        // Commmand given while cover moving, Stop pulse
        ShowDebug("Stopping cover number " + String(CoverPort));
        digitalWrite(PulseRelayPins[PulseRelayPort], PulseRelayInitialStates[PulseRelayPort]);
        String messageString = "P" + String(PulseRelayPort) + "0";
        messageString.toCharArray(messageBuffer, messageString.length() + 1);
        mqttClient.publish(topic_out_pulse, messageBuffer);
      }
    }
    else {
      ShowDebug("No such cover defined.");
    }
  }
  //
  // Command: C + cover number [+ pulse duration in ms]
  //  closes cover:
  //    - sets direction relay: open = normal state
  //    - pulse duration taken from command, if not given, use parameter COVERDELAYTIME
  //    - opens motor relay for duration of pulse
  //  example command: C120000 = close cover 1, use 20000ms (=20s) pulse
  //
  else if (strPayload[0] == 'C') {
    // Cover commando: close
    ShowDebug("Cover command : Close");
    byte CoverPort = strPayload[1] - 48;
    ShowDebug("Cover number " + String(CoverPort));
    if (CoverPort <= NumberOfCovers) {
      int PulseRelayPort = CoverPulse[CoverPort - 1];
      // Check if another command is already running
      if (digitalRead(PulseRelayPins[PulseRelayPort]) == PulseRelayInitialStates[PulseRelayPort]) {
        ShowDebug("Closing cover number " + String(CoverPort));
        // allow for custom pulse time
        long PulseTime = strPayload.substring(2).toInt();
        if (PulseTime == 0) PulseTime = COVERDELAYTIME;
        PulseRelayTimes[PulseRelayPort] = PulseTime;
        // Set direction relay
        digitalWrite(RelayPins[CoverDir[CoverPort - 1]], RelayInitialState[CoverDir[CoverPort - 1]]);
        // Set opening pulse
        digitalWrite(PulseRelayPins[PulseRelayPort], !PulseRelayInitialStates[PulseRelayPort]);
        String messageString = "P" + String(PulseRelayPort) + "1";
        messageString.toCharArray(messageBuffer, messageString.length() + 1);
        mqttClient.publish(topic_out_pulse, messageBuffer);
        PulseActivityTimes[PulseRelayPort] = millis();
        // report state of screen
        CoverState[CoverPort - 1] = 3;  // 3 is closing
        report_state_cover(CoverPort);
        //        messageString = "C" + String(CoverPort);
        //        messageString.toCharArray(messageBuffer, messageString.length() + 1);
        //        mqttClient.publish(topic_out_screen, messageBuffer);
      }
      else {
        ShowDebug("Stopping cover number " + String(CoverPort));
        // Commmand given while cover moving, Stop pulse
        digitalWrite(PulseRelayPins[PulseRelayPort], PulseRelayInitialStates[PulseRelayPort]);
        String messageString = "P" + String(PulseRelayPort) + "0";
        messageString.toCharArray(messageBuffer, messageString.length() + 1);
        mqttClient.publish(topic_out_pulse, messageBuffer);
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
        int PulseRelayPort = CoverPulse[CoverPort - 1];
        // Stop pulse
        digitalWrite(PulseRelayPins[PulseRelayPort], PulseRelayInitialStates[PulseRelayPort]);
        String messageString = "P" + String(PulseRelayPort) + "0";
        messageString.toCharArray(messageBuffer, messageString.length() + 1);
        mqttClient.publish(topic_out_pulse, messageBuffer);
      }
      else {
        ShowDebug("No such cover defined.");
      }
    }
    else {
      ShowDebug("No such cover defined.");
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
      int PulseRelayPort = CoverPulse[CoverPort - 1];
      // Stop pulse
      digitalWrite(PulseRelayPins[PulseRelayPort], PulseRelayInitialStates[PulseRelayPort]);
      String messageString = "P" + String(PulseRelayPort) + "0";
      messageString.toCharArray(messageBuffer, messageString.length() + 1);
      mqttClient.publish(topic_out_pulse, messageBuffer);
    }
    else {
      ShowDebug("No such cover defined.");
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
      String messageString = "pir" + String(pirid) + "on";
      messageString.toCharArray(messageBuffer, messageString.length() + 1);
      mqttClient.publish(topic_out_pir, messageBuffer);
      PreviousDetects[pirid] = true;
    }
  }
  else {
    if (PreviousDetects[pirid]) {
      ShowDebug("Pir " + String(pirid) + " off.");
      String messageString = "pir" + String(pirid) + "off";
      messageString.toCharArray(messageBuffer, messageString.length() + 1);
      mqttClient.publish(topic_out_pir, messageBuffer);
    }
    PreviousDetects[pirid] = false;
  }
}

void setDeviceInfo(char* configtopic) {
  JsonObject device = doc.createNestedObject("device");
  JsonArray identifiers = device.createNestedArray("identifiers");
  identifiers.add(CLIENT_ID);
  JsonArray connections = device.createNestedArray("connections");
  connections.add(serialized("[\"ip\",\"" + String(ip) + "\"]"));
  connections.add(serialized("[\"mac\",\"" + mac2String(mac) + "\"]"));
  device["name"] = DISCOVERY_ID;
  device["model"] = MODEL_ID;
  device["manufacturer"] = MANUFACTURER_ID;
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
  for (int i = 0; i < NumberOfRelays ; i++) {
    doc.clear();
    doc["name"] = "JSON switch" + String(i + 1);
    doc["uniq_id"] = "json_switch" + String(i + 1);
    doc["stat_t"] = state_topic_relays;
    doc["cmd_t"] = topic_in;
    doc["pl_on"] = "R" + String(i) + "0";
    doc["pl_off"] = "R" + String(i) + "1";
    doc["stat_on"] = "on";
    doc["stat_off"] = "off";
    doc["val_tpl"] = "{{value_json.POWER" + String(i) + "}}";
    setDeviceInfo((config_topic_base + "/switch/jsonswitch" + String(i + 1) + "/config").c_str());
  }
  // discovery data for covers
  for (int i = 0; i < NumberOfCovers ; i++ ) {
    doc.clear();
    doc["name"] = "JSON cover" + String(i + 1);
    doc["uniq_id"] = "json_cover" + String(i + 1);
    //    doc["stat_t"] = topic_out_screen;
    doc["pos_t"] = topic_out_screen;
    doc["cmd_t"] = topic_in;
    doc["pl_open"] = "O" + String(i + 1);
    doc["pl_cls"] = "C" + String(i + 1);
    doc["pl_stop"] = "S" + String(i + 1);
    doc["position_open"] = 100;
    doc["position_closed"] = 0;
    doc["set_position_topic"] = topic_in;
    //    doc["stat_open"] = "open";
    //    doc["stat_clsd"] = "closed";
    //    doc["state_opening"] = "opening";
    //    doc["state_closing"] = "closing";
    //    doc["val_tpl"] = "{{value_json.COVER" + String(i + 1) + "}}";
    doc["val_tpl"] = "{{value_json.POSITION" + String(i + 1) + "}}";
    setDeviceInfo((config_topic_base + "/cover/jsoncover" + String(i + 1) + "/config").c_str());
  }
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

  for (int thisPin = 0; thisPin < NumberOfPulseRelays; thisPin++) {
    pinMode(PulseRelayPins[thisPin], OUTPUT);
    ShowDebug("Enabling pulse relay pin " + String(PulseRelayPins[thisPin]));
    digitalWrite(PulseRelayPins[thisPin], PulseRelayInitialStates[thisPin]);
  }

  //  for (int thisButton = 0; thisButton < NumberOfButtons; thisButton++) {
  //    ShowDebug("Button: " + String(ButtonPins[thisButton]));
  //    pinMode(ButtonPins[thisButton], INPUT_PULLUP);
  //  }

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
  //  else {
  //    for (int id = 0; id < NumberOfButtons; id++) {
  //      processButtonDigital(id);
  //    }
  //}

  // and loop.
  mqttClient.loop();
}
