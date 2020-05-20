/*
          <========Arduino Sketch for Arduino Uno Wifi =========>
          Locatie: Tuin
          
          Aansluiting via UTP kabel:

          Pins used:
          0: Serial
          1: Serial

          8: Enable pin for moisture sensor
          
          10: <in gebruik voor W5100>

          50: <in gebruik voor W5100>
          51: <in gebruik voor W5100>
          52: <in gebruik voor W5100>
          53: <in gebruik voor W5100>

          A0: sensor for moisture sensor

          incoming topic: domus/tuin/in

          Arduino Uno Wifi used as MQTT client
          It will connect over Wifi to the MQTT broker and controls digital outputs (LED, relays)
          The topics have the format "domus/tuin/uit" for outgoing messages and
          "domus/tuin/in" for incoming messages.

          The outgoing topics are

          domus/tuin/uit/moisture // Vochtigheid tuingrond (0-100%)

          Here, each relay state is reported using the same syntax as the switch command:
          R<relay number><state>

          There is only one incoming topic:
          domus/tuin/in
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
          - change CLIENT_ID
          - change Mac Address
          - change DISCOVERY ID
          - Change topic base from domus/test with find/replace
          - Change item names
          - Change pin numbers for relays, buttons, pirs
          - Change pin numbers for sensors
          - item_prefix variable

*/

#include "secrets.h"

// parameters to tune memory use
#define UNO_WIFI 1 // use Wifi instead of Ethernet
//#define BMP_present 1 // use BMP280 sensor
//#define DHT_present 1 // use DHT sensor
//#define MQ_present 0 // MQ-x gas sensor
//#define MQ7_present 0 // MQ-7 CO sensor
//#define DS18B20_present 1 // DS18B20 1-wire temperature sensor
//#define LDR_present 1 // LDR sensor
#define MS_present // YL-69 moisture sensor
//#define P1_meter // P1 port smart meter reading
//#define DEBUG 1 // Zet debug mode aan

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

#if defined(MS_present)
#define MS_PIN A0 // Vul hier de pin in van de moisture sensor
#define MS_ENABLE 8 // vul hier de enable pin in
#define SAMPLESIZE 10 // aantal keren meten
#endif

#if defined(P1_meter)
#include "dsmr.h"
#define P1_REQUEST_PIN 49
const int READER_INTERVAL = 5000; // interval to read meter values in ms
using MyData = ParsedData <
               /* FixedValue */ energy_delivered_tariff1,
               /* FixedValue */ energy_delivered_tariff2,
               /* String */ electricity_tariff,
               /* FixedValue */ power_delivered,
               /* FixedValue */ voltage_l1,
               /* FixedValue */ current_l1,
               /* FixedValue */ power_delivered_l1,
               /* TimestampedFixedValue */ gas_delivered
               >;
#endif

// Vul hier de naam in waarmee de Arduino zich aanmeldt bij MQTT, tevens het unique_id bij Home Assistant
#define CLIENT_ID  "domus_tuin"
// Vul hier de naam in waarmee de Arduino zich aanmeldt bij Home Assistant
#define DISCOVERY_ID  "Domus Tuin"
#define MODEL_ID  "Uno Wifi"
#define MANUFACTURER_ID  "Arduino"
String hostname = CLIENT_ID;
// base for Home Assistant MQTT discovery (must be configured in configuration.yaml)
const String config_topic_base = "homeassistant";
// prefix for inidvidual items
const String item_prefix = "tuin";

#if defined(UNO_WIFI)
/////// Wifi Settings ///////
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
#else
// Vul hier het macadres in
uint8_t mac[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x0B};
#endif

// Vul hier de MQTT topic in waar deze arduino naar luistert
const char* topic_in = "domus/tuin/in";

#if defined(MQ7_present)
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
const byte NumberOfRelays = 0;
const byte RelayPins[] = {};
bool RelayInitialState[] = {};
String SwitchNames[] = {};
char* state_topic_relays = "domus/tuin/stat/relay";

// MQTT Discovery lights
// Vul hier het aantal gebruikte relais in en de pinnen waaraan ze verbonden zijn
const byte NumberOfLights = 0;
const byte LightPins[] = {};
bool LightInitialState[] = {};
bool LightBrightness[] = {};
byte LightValue[] = {};
String LightNames[] = {};
const char* state_topic_lights = "domus/tuin/stat/light";
const char* cmd_topic_lights = "domus/tuin/cmd/light";

// MQTT Discovery covers
// Vul hier de gegevens in van de motorsturing voor de screens:
// 2 relais per motor: 1 x richting, 1 x motorpuls
// hiervoor gebruik ik de pulserelais en de normale relais
// de waarden zijn de indices op de onderstaande 'RelayPins' en 'PulseRelayPins' arrays
const byte NumberOfCovers = 0;
byte CoverDir[] = {}; // relay numbers for direction
byte CoverPulse[] = {}; // relay numbers for motor pulses
byte CoverState[] = {}; // 0 = open, 1 = opening, 2 = closed, 3 = closing, 4 = stopped
int CoverPos[] = {}; // position 100 = open
int CoverStart[] = {}; // start position
int CoverSetPos[] = {}; // set position (255 = not set)
String CoverNames[] = {};
long CoverDelay[] = {}; // time to wait for full open or close
const char* state_topic_covers = "domus/tuin/uit/screen"; // Screens (zonwering)

// MQTT Discovery locks
const byte NumberOfLocks = 0;
byte LockPulse[] = {}; // relay numbers for lock pulses (index on PulseRelayPins)
byte LockState[] = {};  // status of locks: 0 = unlocked, 1 = locked
String LockNames[] = {};
long LockDelay[] = {}; // pulse time for locks
const char* state_topic_locks = "domus/tuin/stat/lock"; // Locks (sloten)

// MQTT Discovery pirs (binary_sensors)
const byte NumberOfPirs = 0;
int PirSensors[] = {};
int PirDebounce[] = {}; // debounce time for pir or door sensor
long PirLastActivityTimes[] = {};
static byte lastPirStates[] = {};
bool PirInitialState[] = {};
int PreviousDetects[] = {}; // Statusvariabele PIR sensor
byte PirState[] = {};
String PirNames[] = {};
String PirClasses[] = {};
const char* state_topic_pirs = "domus/tuin/uit/pir";

// MQTT Discovery buttons (device triggers)
const int NumberOfButtons = 0;
int ButtonPins[] = {};
static byte lastButtonStates[] = {};
long lastActivityTimes[] = {};
long LongPressActive[] = {};
String ButtonNames[] = {};
const char* state_topic_buttons = "domus/tuin/uit/button";

// MQTT Discovery sensors (sensors)
const int NumberOfSensors = 2;
const String SensorNames[] = {"Runtime tuin","Vochtigheid grond"};
const String SensorTypes[] = {"TIME","MS"};
String SensorClasses[] = {"",""};
String SensorUnits[] = {"s","%"};
const char* state_topic_sensors = "domus/tuin/uit/sensor";

// Vul hier het aantal pulsrelais in
const int NumberOfPulseRelays = 0; // 
// Vul hier de pins in van het pulserelais.
int PulseRelayPins[] = {};
long PulseActivityTimes[] = {};
// Vul hier de default status in van het pulsrelais (sommige relais vereisen een 0, andere een 1 om te activeren)
// gebruikt 5V YwRobot relay board vereist een 0, 12 volt insteekrelais een 1, SSR relais een 1.
bool PulseRelayInitialStates[] = {HIGH, HIGH, HIGH, HIGH};
// Vul hier de pulsetijden in voor de pulserelais
long int PulseRelayTimes[] = {};
const char* topic_out_pulse = "domus/tuin/uit/pulse";    // Pulserelais t.b.v. deuropener

// Vul hier de uitgaande MQTT topics in
const char* topic_out = "domus/tuin/uit";

#include <domus.h> // this file holds all functions, including setup and loop
