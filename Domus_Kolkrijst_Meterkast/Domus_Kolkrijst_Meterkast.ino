/*
          <========Arduino Sketch for Arduino Mega =========>
          Locatie: Meterkast Kolkrijst
          Macadres: 0x00, 0x01, 0x02, 0x03, 0x05, 0x0B

          Pins used:
          0: Serial
          1: Serial

          4: <in gebruik voor W5100>

          10: <in gebruik voor W5100>
          18: Serial1 TX }
          19: Serial1 RX } in use for P1 Reader
          20: SDA - i2c
          21: SCL - i2c

          22: PIR 
          24: LEDs 
          25: 
          26: 
          28: 
          49: enable pin for P1 reader (not connected)
          50: <in gebruik voor W5100>
          51: <in gebruik voor W5100>
          52: <in gebruik voor W5100>
          53: <in gebruik voor W5100>

          A0:

          incoming topic: domus/meterkast/in

          Arduino Mega with W5100 ethernet shield used as MQTT client
          It will connect over Ethernet to the MQTT broker and controls digital outputs (LED, relays)
          The topics have the format "domus/hobby/uit" for outgoing messages and
          "domus/meterkast/in" for incoming messages.

          The outgoing topics are

          domus/meterkast/uit        // Relaisuitgangen: R<relaisnummer><status>

          Here, each relay state is reported using the same syntax as the switch command:
          R<relay number><state>

          There is only one incoming topic:
          domus/meterkast/in
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

#define VERSION "2023.07.28-1" // version of sketch

// parameters to tune memory use
#define BMP_present 1 // use BMP280 sensor
//#define DHT_present 1 // use DHT sensor
//#define MQ_present 0 // MQ-x gas sensor
//#define MQ7_present 0 // MQ-7 CO sensor
//#define DS18B20_present 1 // DS18B20 1-wire temperature sensor
//#define LDR_present 1 // LDR sensor
#define P1_meter // P1 port smart meter reading
// #define DEBUG 1 // Zet debug mode aan

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

#if defined(MQ_present)
int SmokeSensor = A9;
#endif

// Heartbeat function
byte heartbeatPin = 16;

#if defined(P1_meter)
#include "dsmr.h"
#define P1_REQUEST_PIN 49
const int READER_INTERVAL = 5000; // interval to read meter values in ms
using MyData = ParsedData <
  /* String */ identification,
  /* String */ p1_version,
  /* String */ timestamp,
  /* String */ equipment_id,
  /* FixedValue */ energy_delivered_tariff1,
  /* FixedValue */ energy_delivered_tariff2,
  /* FixedValue */ energy_returned_tariff1,
  /* FixedValue */ energy_returned_tariff2,
  /* String */ electricity_tariff,
  /* FixedValue */ power_delivered,
  /* FixedValue */ power_returned,
  /* uint32_t */ electricity_failures,
  /* uint32_t */ electricity_long_failures,
  /* uint32_t */ electricity_sags_l1,
  /* uint32_t */ electricity_swells_l1,
  /* String */ message_long,
  /* FixedValue */ voltage_l1,
  /* FixedValue */ current_l1,
  /* FixedValue */ power_delivered_l1,
  /* FixedValue */ power_returned_l1,
  /* uint16_t */ gas_device_type,
  /* String */ gas_equipment_id,
  /* TimestampedFixedValue */ gas_delivered >;
#endif

// Vul hier de naam in waarmee de Arduino zich aanmeldt bij MQTT, tevens het unique_id bij Home Assistant
#define CLIENT_ID  "domus_meterkast"
// Vul hier de naam in waarmee de Arduino zich aanmeldt bij Home Assistant
#define DISCOVERY_ID  "Domus Kolkrijst Meterkast"
#define MODEL_ID  "Mega 2560"
#define MANUFACTURER_ID  "Arduino"
String hostname = CLIENT_ID;
// base for Home Assistant MQTT discovery (must be configured in configuration.yaml)
const String config_topic_base = "homeassistant";
// prefix for individual items
const String item_prefix = "meterkast";

// Vul hier het macadres in
uint8_t mac[6] = {0x00, 0x01, 0x02, 0x03, 0x05, 0x0B};

// Vul hier de MQTT topic in waar deze arduino naar luistert
//const char* topic_in = "domus/meterkast/in";

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
const byte NumberOfRelays = 1;
const byte RelayPins[] = {28};
bool RelayInitialState[] = {LOW};
String SwitchNames[] = {"Verwarming"};
char* state_topic_relays = "domus/meterkast/stat/relay";

// MQTT Discovery lights
// Vul hier het aantal gebruikte relais in en de pinnen waaraan ze verbonden zijn
const byte NumberOfLights = 0;
const byte LightPins[] = {24, 26};
bool LightInitialState[] = {HIGH, LOW};
bool LightBrightness[] = {false, false};
byte LightValue[] = {0, 0};
String LightNames[] = {"LEDs", "TL"};
const char* state_topic_lights = "domus/meterkast/stat/light";
const char* cmd_topic_lights = "domus/meterkast/cmd/light";

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
String CoverClasses[] = {"shade", "shade"}; // https://www.home-assistant.io/integrations/cover/
long CoverDelay[] = {28000, 27000}; // time to wait for full open or close
const char* state_topic_covers = "domus/meterkast/uit/screen"; // Screens (zonwering)

// MQTT Discovery locks
const byte NumberOfLocks = 0;
byte LockPulse[] = {0, 1}; // relay numbers for lock pulses (index on PulseRelayPins)
byte LockState[] = {1, 1};  // status of locks: 0 = unlocked, 1 = locked
String LockNames[] = {"*Haldeurslot", "*VoordeurSlot"};
long LockDelay[] = {2000, 250}; // pulse time for locks
const char* state_topic_locks = "domus/meterkast/stat/lock"; // Locks (sloten)

// MQTT Discovery pirs (binary_sensors)
const byte NumberOfPirs = 2;
int PirSensors[] = {22,12};
int PirDebounce[] = {0,0}; // debounce time for pir or door sensor
long PirLastActivityTimes[] = {0,0};
static byte lastPirStates[] = {0,0};
bool PirInitialState[] = {LOW,LOW};
int PreviousDetects[] = {false,false}; // Statusvariabele PIR sensor
byte PirState[] = {0,0};
String PirNames[] = {"PIR","Deurbel"};
String PirClasses[] = {"motion","sound"};
const char* state_topic_pirs = "domus/meterkast/uit/pir";

// MQTT Discovery buttons (device triggers)
const int NumberOfButtons = 0;
int ButtonPins[] = {12};
static byte lastButtonStates[] = {0};
long lastActivityTimes[] = {0};
long LongPressActive[] = {0};
String ButtonNames[] = {"Deurbel"};
const char* state_topic_buttons = "domus/meterkast/uit/button";

// MQTT Discovery sensors (sensors)
const int NumberOfSensors = 13;
// const int NumberOfSensors = 3;
// String SensorNames[] = {"Temperatuur", "Runtime", "Luchtdruk"};
// String SensorTypes[] = {"BMP-T", "TIME", "BMP-P"};
// String SensorClasses[] = {"temperature", "", "pressure"};
// String SensorUnits[] = {"°C", "s", "mbar"};
const char* const SensorNames[] = {"Runtime meterkast", "Energieverbruik laag", "Energieverbruik hoog", "Energietarief", "Energieverbruik", "Netspanning", "Stroomsterkte", "Gasverbruik", "Luchtdruk", "Temperatuur", "Energie teruglevering laag", "Energie teruglevering hoog", "Energie teruglevering"};
const char* const SensorTypes[] = {"TIME", "P1_en_t1", "P1_en_t2", "P1_ta", "P1_pd", "P1_v1", "P1_c1", "P1_gas", "BMP-P", "BMP-T", "P1_rt_t1", "P1_rt_t2", "P1_pr"};
const char* const SensorClasses[] = {"", "energy", "energy", "", "power", "voltage", "current", "gas", "pressure", "temperature", "energy", "energy", "power"};
const char* const StateClasses[] = {"total_increasing", "total_increasing", "total_increasing", "", "measurement", "measurement", "measurement", "total_increasing", "measurement", "measurement", "total_increasing", "total_increasing", "measurement"};
const char* const SensorUnits[] = {"s", "kWh", "kWh", "", "W", "V", "A", "m³", "mBar", "°C", "kWh", "kWh", "W"};
const char* state_topic_sensors = "domus/meterkast/uit/sensor";

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
const char* topic_out_pulse = "domus/meterkast/uit/pulse";    // Pulserelais t.b.v. deuropener

// Vul hier de uitgaande MQTT topics in
const char* topic_out = "domus/meterkast/uit";

#include <domus.h> // this file holds all functions, including setup and loop
