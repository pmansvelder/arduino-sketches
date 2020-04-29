/*
          <========Arduino Sketch for Arduino Mega =========>
          Locatie: Zolder
          Macadres: 0x00, 0x01, 0x02, 0x03, 0x05, 0x0B

          Pins used:
          0: Serial
          1: Serial

          4: <in gebruik voor W5100>

          10: <in gebruik voor W5100>

          20: SDA - i2c
          21: SCL - i2c

          50: <in gebruik voor W5100>
          51: <in gebruik voor W5100>
          52: <in gebruik voor W5100>
          53: <in gebruik voor W5100>

          A0:

          incoming topic: domus/zolder/in

          Arduino Mega with W5100 ethernet shield used as MQTT client
          It will connect over Ethernet to the MQTT broker and controls digital outputs (LED, relays)
          The topics have the format "domus/hobby/uit" for outgoing messages and
          "domus/zolder/in" for incoming messages.

          The outgoing topics are

          domus/zolder/uit        // Relaisuitgangen: R<relaisnummer><status>

          Here, each relay state is reported using the same syntax as the switch command:
          R<relay number><state>

          There is only one incoming topic:
          domus/zolder/in
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
          - Change item names -> done
          - Change pin numbers for relays, buttons, pirs -> done
          - Change pin numbers for sensors -> done
          - item_prefix variable -> done


*/

#include "secrets.h"

// parameters to tune memory use
#define BMP_present 1 // use BMP280 sensor
//#define DHT_present 1 // use DHT sensor
//#define MQ_present 0 // MQ-x gas sensor
//#define MQ7_present 0 // MQ-7 CO sensor
//#define DS18B20_present 1 // DS18B20 1-wire temperature sensor
//#define LDR_present 1 // LDR sensor
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

// Vul hier de naam in waarmee de Arduino zich aanmeldt bij MQTT, tevens het unique_id bij Home Assistant
#define CLIENT_ID  "domus_zolder"
// Vul hier de naam in waarmee de Arduino zich aanmeldt bij Home Assistant
#define DISCOVERY_ID  "Domus Mega Zolder"
#define MODEL_ID  "Mega 2560"
#define MANUFACTURER_ID  "Arduino"
String hostname = CLIENT_ID;
// base for Home Assistant MQTT discovery (must be configured in configuration.yaml)
const String config_topic_base = "homeassistant";
// prefix for inidvidual items
const String item_prefix = "zolder";

// Vul hier het macadres in
uint8_t mac[6] = {0x00, 0x01, 0x02, 0x03, 0x05, 0x0B};

// Vul hier de MQTT topic in waar deze arduino naar luistert
const char* topic_in = "domus/zolder/in";

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
const byte RelayPins[] = {5, 6, 23, 25};
bool RelayInitialState[] = {LOW, LOW, LOW, LOW};
String SwitchNames[] = {"*Mediaplayer Keuken", "*CV-ketel", "*Screen keuken", "*Screen Huiskamer"};
char* state_topic_relays = "domus/zolder/stat/relay";

// MQTT Discovery lights
// Vul hier het aantal gebruikte relais in en de pinnen waaraan ze verbonden zijn
const byte NumberOfLights = 0;
const byte LightPins[] = {30, 31, 2};
bool LightInitialState[] = {HIGH, HIGH, HIGH};
bool LightBrightness[] = {false, false, true};
byte LightValue[] = {0, 0, 0};
String LightNames[] = {"*Keuken", "*Plafondlamp", "*Buttonleds"};
const char* state_topic_lights = "domus/zolder/stat/light";
const char* cmd_topic_lights = "domus/zolder/cmd/light";

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
const char* state_topic_covers = "domus/zolder/uit/screen"; // Screens (zonwering)

// MQTT Discovery locks
const byte NumberOfLocks = 0;
byte LockPulse[] = {0, 1}; // relay numbers for lock pulses (index on PulseRelayPins)
byte LockState[] = {1, 1};  // status of locks: 0 = unlocked, 1 = locked
String LockNames[] = {"*Haldeurslot", "*VoordeurSlot"};
long LockDelay[] = {2000, 250}; // pulse time for locks
const char* state_topic_locks = "domus/zolder/stat/lock"; // Locks (sloten)

// MQTT Discovery pirs (binary_sensors)
const byte NumberOfPirs = 0;
int PirSensors[] = {19, 28, 29};
int PirDebounce[] = {0, 0, 0}; // debounce time for pir or door sensor
long PirLastActivityTimes[] = {};
static byte lastPirStates[] = {};
bool PirInitialState[] = {LOW, LOW, LOW};
int PreviousDetects[] = {false, false, false}; // Statusvariabele PIR sensor
byte PirState[] = {0, 0, 0};
String PirNames[] = {"*PIR Hal", "*PIR Keuken", "*Voordeur"};
String PirClasses[] = {"motion", "motion", "door"};
const char* state_topic_pirs = "domus/zolder/uit/pir";

// MQTT Discovery buttons (device triggers)
const int NumberOfButtons = 0;
int ButtonPins[] = {11, 12, 9};
static byte lastButtonStates[] = {0, 0, 0};
long lastActivityTimes[] = {0, 0, 0};
long LongPressActive[] = {0, 0, 0};
String ButtonNames[] = {"*Knop Keuken", "*Keuken", "*Voordeur"};
const char* state_topic_buttons = "domus/zolder/uit/button";

// MQTT Discovery sensors (sensors)
const int NumberOfSensors = 3;
String SensorNames[] = {"Temperatuur zolder", "Runtime zolder", "Luchtdruk zolder"};
String SensorTypes[] = {"BMP-T", "TIME", "BMP-P"};
String SensorClasses[] = {"temperature", "timestamp", "pressure"};
String SensorUnits[] = {"°C", "s", "mbar"};
const char* state_topic_sensors = "domus/zolder/uit/sensor";

// Vul hier het aantal pulsrelais in
const int NumberOfPulseRelays = 0; // 0 = haldeur, 1 = voordeur, 2 = screen keuken, 3 = screen huiskamer
// Vul hier de pins in van het pulserelais.
int PulseRelayPins[] = {8, 7, 22, 24};
long PulseActivityTimes[] = {0, 0, 0, 0};
// Vul hier de default status in van het pulsrelais (sommige relais vereisen een 0, andere een 1 om te activeren)
// gebruikt 5V YwRobot relay board vereist een 0, 12 volt insteekrelais een 1, SSR relais een 1.
bool PulseRelayInitialStates[] = {HIGH, HIGH, HIGH, HIGH};
// Vul hier de pulsetijden in voor de pulserelais
long int PulseRelayTimes[] = {LockDelay[0], LockDelay[1], CoverDelay[0], CoverDelay[1]};
const char* topic_out_pulse = "domus/zolder/uit/pulse";    // Pulserelais t.b.v. deuropener

// Vul hier de uitgaande MQTT topics in
const char* topic_out = "domus/zolder/uit";

#include <domus.h> // this file holds all functions, including setup and loop
