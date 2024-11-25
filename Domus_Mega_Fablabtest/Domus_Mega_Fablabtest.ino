/*
          <========Arduino Sketch for Arduino Mega =========>
          Locatie: test
          Macadres: A8:61:0A:03:04:09
          prefix voor Arduino AG (bestaat niet meer): A8:61:0A

                                      +-----+
         +----[PWR]-------------------| USB |--+
         |                            +-----+  |
         |           GND/RST2  [ ] [ ]         |
         |         MOSI2/SCK2  [ ] [ ]  SCL[ ] |   D0
         |            5V/MISO2 [ ] [ ]  SDA[ ] |   D1
         |                             AREF[ ] |
         |                              GND[ ] |
         | [ ]N/C                        13[ ]~|   B7
         | [ ]IOREF                      12[ ]~|   B6
         | [ ]RST                        11[ ]~|   B5
         | [ ]3V3      +----------+      10[ ]~|   B4
         | [ ]5v       | ARDUINO  |       9[ ]~|   H6
         | [ ]GND      |   MEGA   |       8[ ]~|   H5
         | [ ]GND      +----------+            |
         | [ ]Vin                         7[ ]~|   H4
         |                                6[ ]~|   H3
         | [ ]A0                          5[ ]~|   E3
         | [ ]A1                          4[ ]~|   G5
         | [ ]A2                     INT5/3[ ]~|   E5
         | [ ]A3                     INT4/2[ ]~|   E4
         | [ ]A4                       TX>1[ ]~|   E1
         | [ ]A5                       RX<0[ ]~|   E0
         | [ ]A6                               |   
         | [ ]A7                     TX3/14[ ] |   J1
         |                           RX3/15[ ] |   J0
         | [ ]A8                     TX2/16[ ] |   H1         
         | [ ]A9                     RX2/17[ ] |   H0
         | [ ]A10               TX1/INT3/18[ ] |   D3         
         | [ ]A11               RX1/INT2/19[ ] |   D2
         | [ ]A12           I2C-SDA/INT1/20[ ] |   D1         
         | [ ]A13           I2C-SCL/INT0/21[ ] |   D0
         | [ ]A14                              |            
         | [ ]A15                              |   Ports:
         |                RST SCK MISO         |    22=A0  23=A1   
         |         ICSP   [ ] [ ] [ ]          |    24=A2  25=A3   
         |                [ ] [ ] [ ]          |    26=A4  27=A5   
         |                GND MOSI 5V          |    28=A6  29=A7   
         | G                                   |    30=C7  31=C6   
         | N 5 5 4 4 4 4 4 3 3 3 3 3 2 2 2 2 5 |    32=C5  33=C4   
         | D 2 0 8 6 4 2 0 8 6 4 2 0 8 6 4 2 V |    34=C3  35=C2   
         |         ~ ~                         |    36=C1  37=C0   
         | @ # # # # # # # # # # # # # # # # @ |    38=D7  39=G2    
         | @ # # # # # # # # # # # # # # # # @ |    40=G1  41=G0   
         |           ~                         |    42=L7  43=L6   
         | G 5 5 4 4 4 4 4 3 3 3 3 3 2 2 2 2 5 |    44=L5  45=L4   
         | N 3 1 9 7 5 3 1 9 7 5 3 1 9 7 5 3 V |    46=L3  47=L2   
         | D                                   |    48=L1  49=L0    SPI:
         |                                     |    50=B3  51=B2     50=MISO 51=MOSI
         |     2560                ____________/    52=B1  53=B0     52=SCK  53=SS 
          \_______________________/         
         
         http://busyducks.com/ascii-art-arduinos

          Pins used:
          0: Serial
          1: Serial
          2:
          3: 
          4: <in gebruik voor W5100>
          5: 
          6: 
          7: 
          8:
          9: 
          10: <in gebruik voor W5100>
          11: 
          12:
          13: 
          16: NE555 Heartbeat module

          A0: input for humidity sensor

          20: SDA voor i2C
          21: SCL voor i2C
          22: SSRelay 1
          24: SSRelay 2
          26: SSRelay 3
          28: SSRelay 4
          30: 
          31: 
          32: 
          35: 

          50: <in gebruik voor W5100>
          51: <in gebruik voor W5100>
          52: <in gebruik voor W5100>
          53: <in gebruik voor W5100>

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

#include "secrets.h"

#define VERSION "2024.01.14-1"  // version of sketch

// parameters to tune memory use
// #define BMP_present 1 // use BMP280 sensor
// #define DHT_present 1 // use DHT sensor
// #define MQ_present 0 // MQ-x gas sensor
// #define MQ7_present 0 // MQ-7 CO sensor
// #define MS_present // YL-69 moisture sensor
// #define DS18B20_present 1 // DS18B20 1-wire temperature sensor
// #define LDR_present 1 // LDR sensor
// #define P1_meter // P1 port smart meter reading
// #define MEMORY // report free memory
// #define RECYCLE // reboot after x minutes due to instability
#define LIGHTS
// #define TRIGGERS
// #define COVERS
// #define LOCKS
// #define DEBUG 1 // Zet debug mode aan

#if defined(DHT_present)
#include <DHT.h>
#define DHT_PIN 3  // Vul hier de pin in van de DHT11/22 sensor
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
#include <Adafruit_BMP280.h>  // Adafruit BMP280 library
Adafruit_BMP280 bmp;          // I2C: SDA=20, SCL=21
#endif

#if defined(MQ_present)
int SmokeSensor = A9;
#endif

// Heartbeat function
byte heartbeatPin = 16;

#if defined(MS_present)
#define MS_PIN A0      // Vul hier de pin in van de moisture sensor
#define MS_ENABLE 35   // vul hier de enable pin in
#define SAMPLESIZE 10  // aantal keren meten
#endif

// Vul hier de naam in waarmee de Arduino zich aanmeldt bij MQTT, tevens het unique_id bij Home Assistant
#define CLIENT_ID "domus_test"
// Vul hier de naam in waarmee de Arduino zich aanmeldt bij Home Assistant
#define DISCOVERY_ID "Domus test"
#define MODEL_ID "Mega 2560"
#define MANUFACTURER_ID "Arduino"
String hostname = CLIENT_ID;
// base for Home Assistant MQTT discovery (must be configured in configuration.yaml)
const String config_topic_base = "homeassistant";
// prefix for inidvidual items
const String item_prefix = "test";

// Vul hier het macadres in A8:61:0A
uint8_t mac[6] = { 0xA8, 0x61, 0x0A, 0x03, 0x04, 0x09 };

// Vul hier de MQTT topic in waar deze arduino naar luistert
//const char* topic_in = "domus/test/in";

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
const byte RelayPins[] = { 28 };
bool RelayInitialState[] = { LOW };
String SwitchNames[] = { "Stopcontact" };
char* state_topic_relays = "domus/test/stat/relay";

// MQTT Discovery lights
// Vul hier het aantal gebruikte relais in en de pinnen waaraan ze verbonden zijn
const byte NumberOfLights = 1;
const byte LightPins[] = { 22, 24, 26 };
bool LightInitialState[] = { LOW, LOW, LOW };
bool LightBrightness[] = { false, false, false };
byte LightValue[] = { 0, 0, 0 };
String LightNames[] = { "test", "Garage", "Buitenlamp" };
const char* state_topic_lights = "domus/test/stat/light";
const char* cmd_topic_lights = "domus/test/cmd/light";

// MQTT Discovery covers
// Vul hier de gegevens in van de motorsturing voor de screens:
// 2 relais per motor: 1 x richting, 1 x motorpuls
// hiervoor gebruik ik de pulserelais en de normale relais
// de waarden zijn de indices op de onderstaande 'RelayPins' en 'PulseRelayPins' arrays
const byte NumberOfCovers = 0;
byte CoverDir[] = { 2, 3 };        // relay numbers for direction
byte CoverPulse[] = { 2, 3 };      // relay numbers for motor pulses
byte CoverState[] = { 0, 0 };      // 0 = open, 1 = opening, 2 = closed, 3 = closing, 4 = stopped
int CoverPos[] = { 100, 100 };     // position 100 = open
int CoverStart[] = { 100, 100 };   // start position
int CoverSetPos[] = { 255, 255 };  // set position (255 = not set)
String CoverNames[] = { "*Screen Keuken", "*Screen Huiskamer" };
String CoverClasses[] = {};                                  // https://www.home-assistant.io/integrations/cover/
long CoverDelay[] = { 28000, 27000 };                        // time to wait for full open or close
const char* state_topic_covers = "domus/test/uit/screen";  // Screens (zonwering)

// MQTT Discovery locks
const byte NumberOfLocks = 0;
byte LockPulse[] = { 0, 1 };  // relay numbers for lock pulses (index on PulseRelayPins)
byte LockState[] = { 1, 1 };  // status of locks: 0 = unlocked, 1 = locked
String LockNames[] = { "*Haldeurslot", "*VoordeurSlot" };
long LockDelay[] = { 2000, 250 };                          // pulse time for locks
const char* state_topic_locks = "domus/test/stat/lock";  // Locks (sloten)

// MQTT Discovery pirs (binary_sensors)
const byte NumberOfPirs = 1;
const byte PirSensors[] = { 52 };
const byte PirDebounce[] = { 0 };  // debounce time for pir or door sensor
long PirLastActivityTimes[] = { 0 };
static byte lastPirStates[] = { 0 };
bool PirInitialState[] = { LOW };
int PreviousDetects[] = { false };  // Statusvariabele PIR sensor
byte PirState[] = { 0 };
String PirNames[] = { "test" };
String PirClasses[] = { "motion" };
const char* state_topic_pirs = "domus/test/uit/pir";

// MQTT Discovery buttons (device triggers)
const int NumberOfButtons = 0;
int ButtonPins[] = {};
static byte lastButtonStates[] = { 0, 0 };
long lastActivityTimes[] = { 0, 0 };
long LongPressActive[] = { 0, 0 };
String ButtonNames[] = {};
const char* state_topic_buttons = "domus/test/uit/button";

// MQTT Discovery sensors (sensors)
const int NumberOfSensors = 1;
const char* const SensorNames[] = { "Runtime test" };
const char* const SensorTypes[] = { "TIME" };
const char* const SensorClasses[] = { "" };
const char* const StateClasses[] = { "total_increasing" };
const char* const SensorUnits[] = { "s" };
const char* state_topic_sensors = "domus/test/uit/sensor";

// Vul hier het aantal pulsrelais in
const int NumberOfPulseRelays = 0;  //
// Vul hier de pins in van het pulserelais.
int PulseRelayPins[] = {};
long PulseActivityTimes[] = { 0, 0, 0, 0 };
// Vul hier de default status in van het pulsrelais (sommige relais vereisen een 0, andere een 1 om te activeren)
// gebruikt 5V YwRobot relay board vereist een 0, 12 volt insteekrelais een 1, SSR relais een 1.
bool PulseRelayInitialStates[] = { HIGH, HIGH, HIGH, HIGH };
String PulseSwitchNames[] = {};
// Vul hier de pulsetijden in voor de pulserelais
long int PulseRelayTimes[] = { LockDelay[0], LockDelay[1], CoverDelay[0], CoverDelay[1] };
// const char* topic_out_pulse = "domus/test/uit/pulse";  // Pulserelais t.b.v. deuropener
const char* state_topic_pulserelays = "domus/test/stat/pulserelay";

// Vul hier de uitgaande MQTT topics in
const char* topic_out = "domus/test/uit";

#include <domus.h>  // this file holds all functions, including setup and loop
