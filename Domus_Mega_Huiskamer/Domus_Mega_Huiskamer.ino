/*
          <========Arduino Sketch for Arduino Mega =========>
          Locatie: Huiskamer
          Macadres: 00:01:02:03:04:06

          Aansluiting via UTP kabel:
          - 5V : bruin
          - GND: wit

          Pins used:
          2     LED for button hallway
          3     Relay port R8: 12v relay for TV
          4     < reserved for Ethernet Shield >
          5     Relay port R9: 12v relay for stereo
          8     Relay port R1: 12v relay for lamp living room
          9     Relay port R0: 12v relay for radiator valve
          10    < reserved for Ethernet Shield >
          11
          12
          13
          14    Relay port R6: SSR Tuin 1
          15    Relay port R7: SSR Tuin 2
          16    OneWire 18B20 sensor
          17    PIR sensor tuin
          20    I2c SDA
          21    I2c SCL
          22    Relay port R4: SSR for lamp bed chamber
          23    Button bed chamber
          24    DHT22 living room
          25    Relay port R5 :Buzzer living room
          35    Button hallway
          36 .  Relay port R2: SSR for lamp hallway
          37    Relay port R3: heating
          38    PIR sensor living room

          50: <in gebruik voor W5100>
          51: <in gebruik voor W5100>
          52: <in gebruik voor W5100>
          53: <in gebruik voor W5100>

          A0:   MQ7 sensor input pin
          A9    Smoke Sensor MQ-2 living room
          A10   LDR living room

          incoming topic: domus/hk/in

          Arduino Mega with W5100 ethernet shield used as MQTT client
          It will connect over Ethernet to the MQTT broker and controls digital outputs (LED, relays)
          The topics have the format "domus/hobby/uit" for outgoing messages and
          "domus/hk/in" for incoming messages.

          The outgoing topics are

          domus/hk/uit        // Relaisuitgangen: R<relaisnummer><status>

          Here, each relay state is reported using the same syntax as the switch command:
          R<relay number><state>

          There is only one incoming topic:
          domus/hk/in
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
          - Change CLIENT_ID
          - change Mac Address
          - change DISCOVERY ID
          - Change topic base from domus/hobby with find/replace
          - Change item names
          - Change pin numbers for relays, buttons, pirs
          - Change pin numbers for sensors
          - item_prefix variable


*/

#include "secrets.h"

// parameters to tune memory use
//#define BMP_present 1 // use BMP280 sensor
#define DHT_present 1 // use DHT sensor
#define MQ_present 0 // MQ-x gas sensor
//#define MQ7_present 0 // MQ-7 CO sensor
#define DS18B20_present 1 // DS18B20 1-wire temperature sensor
#define LDR_present 1 // LDR sensor
//#define DEBUG 1 // Zet debug mode aan

#if defined(DHT_present)
#include <DHT.h>
#define DHT_PIN 24 // Vul hier de pin in van de DHT11/22 sensor
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
#define CLIENT_ID  "domus_huiskamer"
// Vul hier de naam in waarmee de Arduino zich aanmeldt bij Home Assistant
#define DISCOVERY_ID  "Domus Huiskamer"
#define MODEL_ID  "Mega 2560"
#define MANUFACTURER_ID  "Arduino"
String hostname = CLIENT_ID;
// base for Home Assistant MQTT discovery (must be configured in configuration.yaml)
const String config_topic_base = "homeassistant";
// prefix for inidvidual items
const String item_prefix = "hk";

// Vul hier het macadres in
uint8_t mac[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x06};

// Vul hier de MQTT topic in waar deze arduino naar luistert
const char* topic_in = "domus/hk/in";

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

// MQTT Discovery relays
// Vul hier het aantal gebruikte relais in en de pinnen waaraan ze verbonden zijn
const byte NumberOfRelays = 6;
const byte RelayPins[] = {9, 37, 25, 3, 5, 22};
bool RelayInitialState[] = {LOW, LOW, LOW, LOW, LOW, LOW};
String SwitchNames[] = {"Stopcontact Huiskamer", "Verwarming", "Zoemer Huiskamer", "LoggerTV", "Stereo", "Slaapkamer"};
char* state_topic_relays = "domus/hk/stat/relay";

// MQTT Discovery lights
// Vul hier het aantal gebruikte relais in en de pinnen waaraan ze verbonden zijn
const byte NumberOfLights = 5;
const byte LightPins[] = {8, 36, 14, 15, 2};
bool LightInitialState[] = {LOW, LOW, LOW, LOW, LOW};
bool LightBrightness[] = {false, false, false, false, true};
byte LightValue[] = {0, 0, 0, 0, 0};
String LightNames[] = {"Staande lamp", "Lamp hal", "Floodlight tuin", "Tuinlamp huis", "Buttonleds huiskamer"};
const char* state_topic_lights = "domus/hk/stat/light";
const char* cmd_topic_lights = "domus/hk/cmd/light";

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
int CoverSetPos[] = {255, 255}; // set position (255 = not set)
String CoverNames[] = {"*Screen Keuken", "*Screen Huiskamer"};
long CoverDelay[] = {28000, 27000}; // time to wait for full open or close
const char* state_topic_covers = "domus/hk/uit/screen"; // Screens (zonwering)

// MQTT Discovery locks
const byte NumberOfLocks = 0;
byte LockPulse[] = {0, 1}; // relay numbers for lock pulses (index on PulseRelayPins)
byte LockState[] = {1, 1};  // status of locks: 0 = unlocked, 1 = locked
String LockNames[] = {"*Haldeurslot", "*VoordeurSlot"};
long LockDelay[] = {250, 2000}; // pulse time for locks
const char* state_topic_locks = "domus/hk/stat/lock"; // Locks (sloten)

// MQTT Discovery pirs (binary_sensors)
const byte NumberOfPirs = 2;
int PirSensors[] = {38, 17};
int PirDebounce[] = {0, 0}; // debounce time for pir or door sensor
int PreviousDetects[] = {false, false}; // Statusvariabele PIR sensor
byte PirState[] = {0, 0};
String PirNames[] = {"PIR huiskamer", "PIR Tuin"};
String PirClasses[] = {"motion", "motion"};
const char* state_topic_pirs = "domus/hk/uit/pir";

// MQTT Discovery buttons (device triggers)
const int NumberOfButtons = 2;
int ButtonPins[] = {23, 35};
static byte lastButtonStates[] = {0, 0};
long lastActivityTimes[] = {0, 0};
long LongPressActive[] = {0, 0};
String ButtonNames[] = {"Knop slaapkamer", "Knop hal"};
const char* state_topic_buttons = "domus/hk/uit/button";

// MQTT Discovery sensors (sensors)
const int NumberOfSensors = 7;
String SensorNames[] = {"Temperatuur huiskamer", "Luchtvochtigheid huiskamer", "Gevoelstemperatuur huiskamer", "VOS sensor huiskamer", "Lichtsterkte huiskamer", "Temperatuur tuin", "Runtime huiskamer"};
String SensorTypes[] = {"DHT-T", "DHT-H", "DHT-I", "MQ2", "LDR", "DS18B20", "TIME"};
String SensorClasses[] = {"temperature", "humidity", "temperature", "", "illuminance", "temperature", "timestamp"};
String SensorUnits[] = {"°C", "%", "°C", "%" , "lux", "°C", "s"};
const char* state_topic_sensors = "domus/hk/uit/sensor";

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
const char* topic_out_pulse = "domus/hk/uit/pulse";    // Pulserelais t.b.v. deuropener

// Vul hier de uitgaande MQTT topics in
const char* topic_out = "domus/hk/uit";

#include <domus.h> // this file holds all functions, including setup and loop
