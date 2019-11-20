/*
          <========Arduino Sketch for ESP01 =========>
          Locatie: Meterkast
          Macadres: xx:xx:xx:xx:xx:xx
          Pins used:
          2: PWM voor LEDs
          3: DHT sensor
          4: <in gebruik voor W5100>
          5: Relay 0
          6: Relay 1
          7: Relay 2
          8: Relay 3 (Pulse, Deuropener)
          9:  Button 2 (keuken)
          10: <in gebruik voor W5100>
          11: Button 0 (huiskamer)
          12: Button 1 (huiskamer)
          13:
          30: SSR Relais voor keukenlamp
          31: SSR Relais voor plafondlamp huiskamer

          incoming topic: domus/mk/in

          ESP01 used as MQTT client
          It will connect over Wifi to the MQTT broker and controls digital outputs (LED, relays)
          The topics have the format "domus/hk/uit" for outgoing messages and
          "domus/hk/in" for incoming messages.
          As the available memory of a UNO  with Ethernetcard is limited,
          I have kept the topics short
          Also, the payloads  are kept short
          The outgoing topics are

          domus/hk/uit        // Relaisuitgangen: R<relaisnummer><status>
          domus/hk/uit/rook   // MQ-2 gas & rookmelder, geconverteerd naar 0-100%
          domus/hk/uit/licht  // LDR-melder: 0=licht, 1=donker
          domus/hk/uit/temp   // DHT-11 temperatuursensor
          domus/hk/uit/warmte // DHT-11 gevoelstemperatuur
          domus/hk/uit/vocht  // DHT-11 luchtvochtigheid
          domus/ex/uit/deur   // Pulserelais t.b.v. deuropener

          Here, each relay state is reported using the same syntax as the switch command:
          R<relay number><state>

          There is only one incoming topic:
          domus/hk/in
          The payload here determines the action:
          STAT - Report the status of all relays (0-9)
          AON - turn all the relays on
          AOF - turn all the relays off
          2 - Publish the IP number of the client
          R<relay number><state> - switch relay into specified state (0=off, 1=on)
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

          ToDo:
          - add extra button on input A5(19) => done
          - use output 17 as a PWM channel for the button LEDs => done, used A9
          - implement short/long press for buttons => done

          Adapted 24-02-2018 by Peter Mansvelder:
          - added sensors for light and smoke (MQ-2), reporting on output topics
          - added alarm function for smoke with buzzer
          - added pulse relay

          Adapted 22-07-2018 by Peter Mansvelder
          - added smart meter P1 input

*/

#include <ESP8266WiFi.h>
#include "secrets.h"
#include "PubSubClient.h" //PubSubClient.h Library from Knolleary
#include <dsmr.h>

// Vul hier de naam in waarmee de Arduino zich aanmeldt bij MQTT
#define CLIENT_ID  "domus_esp_meterkast"

// Vul hier het interval in waarmee sensorgegevens worden verstuurd op MQTT
#define PUBLISH_DELAY 5000 // that is 3 seconds interval
#define DEBOUNCE_DELAY 150
#define LONGPRESS_TIME 450

//DSMR stuff
using MyData = ParsedData <
               /* String */         identification
               /* String */        , p1_version
               /* String */        , timestamp
               /* String */        , equipment_id
               /* FixedValue */    , energy_delivered_tariff1
               /* FixedValue */    , energy_delivered_tariff2
               /* FixedValue */    , energy_returned_tariff1
               /* FixedValue */    , energy_returned_tariff2
               /* String */        , electricity_tariff
               /* FixedValue */    , power_delivered
               /* FixedValue */    , power_returned
               //  /* FixedValue */    ,electricity_threshold
               //  /* uint8_t */       ,electricity_switch_position
               //  /* uint32_t */      ,electricity_failures
               //  /* uint32_t */      ,electricity_long_failures
               //  /* String */        ,electricity_failure_log
               //  /* uint32_t */      ,electricity_sags_l1
               //  /* uint32_t */      ,electricity_sags_l2
               //  /* uint32_t */      ,electricity_sags_l3
               //  /* uint32_t */      ,electricity_swells_l1
               //  /* uint32_t */      ,electricity_swells_l2
               //  /* uint32_t */      ,electricity_swells_l3
               //  /* String */        ,message_short
               //  /* String */        ,message_long
               /* FixedValue */    , voltage_l1
               /* FixedValue */    , voltage_l2
               /* FixedValue */    , voltage_l3
               /* FixedValue */    , current_l1
               /* FixedValue */    , current_l2
               /* FixedValue */    , current_l3
               /* FixedValue */    , power_delivered_l1
               /* FixedValue */    , power_delivered_l2
               /* FixedValue */    , power_delivered_l3
               /* FixedValue */    , power_returned_l1
               /* FixedValue */    , power_returned_l2
               /* FixedValue */    , power_returned_l3
               /* uint16_t */      , gas_device_type
               /* String */        , gas_equipment_id
               //  /* uint8_t */       ,gas_valve_position
               /* TimestampedFixedValue */ , gas_delivered
               //  /* uint16_t */      ,thermal_device_type
               //  /* String */        ,thermal_equipment_id
               //  /* uint8_t */       ,thermal_valve_position
               //  /* TimestampedFixedValue */ ,thermal_delivered
               //  /* uint16_t */      ,water_device_type
               //  /* String */        ,water_equipment_id
               //  /* uint8_t */       ,water_valve_position
               //  /* TimestampedFixedValue */ ,water_delivered
               //  /* uint16_t */      ,slave_device_type
               //  /* String */        ,slave_equipment_id
               //  /* uint8_t */       ,slave_valve_position
               //  /* TimestampedFixedValue */ ,slave_delivered
               >;

MyData P1data;
P1Reader reader(&Serial, 0);
//

String hostname = CLIENT_ID;

// Vul hier de MQTT topic in waar deze arduino naar luistert
const char* topic_in = "domus/espmk/in";

// Vul hier de uitgaande MQTT topics in
const char* topic_out = "domus/espmk/uit";
const char* topic_out_meter_voltage = "domus/espmk/uit/meter/voltage";
const char* topic_out_meter_tariff = "domus/espmk/uit/meter/tarief";
const char* topic_out_meter_high = "domus/espmk/uit/meter/meterhigh";
const char* topic_out_meter_low = "domus/espmk/uit/meter/meterlow";
const char* topic_out_meter_power = "domus/espmk/uit/meter/power";
const char* topic_out_meter_gas = "domus/espmk/uit/meter/gas";

bool statusKD = HIGH;
bool statusBD = HIGH;
bool statusGD = HIGH;
bool relaystate1 = LOW;
bool pir = LOW;
bool startsend = HIGH;// flag for sending at startup
bool debug = true;
int lichtstatus; //contains LDR reading

const char* ssid     = SECRET_SSID;
const char* password = SECRET_PASS;

char messageBuffer[100];
char topicBuffer[100];
String ip = "";

WiFiClient espClient;
PubSubClient mqttClient;

long previousMillis;

// variables for meter reading
char input; // incoming serial data (byte)
bool readnextLine = false;
#define BUFSIZE 75
char buffer[BUFSIZE]; //Buffer for serial data to find \n .
int bufpos = 0;
long mEVLT = 0; //Meter reading Electrics - consumption low tariff
long mEVHT = 0; //Meter reading Electrics - consumption high tariff
long mEAV = 0;  //Meter reading Electrics - Actual consumption
long mG = 0;   //Meter reading Gas
int Tariff = 1; // Current tariff
long Voltage = 0; // Voltage

void callback(char* topic, byte * payload, unsigned int length) {
  char msgBuffer[20];
  // I am only using one ascii character as command, so do not need to take an entire word as payload
  // However, if you want to send full word commands, uncomment the next line and use for string comparison
  payload[length] = '\0'; // terminate string with 0
  String strPayload = String((char*)payload);  // convert to string
  int RelayPort;
  int RelayValue;

  if (strPayload == "IP")  {

    // 'Show IP' commando
    mqttClient.publish(topic_out, ip.c_str());// publish IP nr
  }
  else {

    // Onbekend commando
    mqttClient.publish(topic_out, "Unknown command");
  }
}

void setup() {
  Serial.begin(115200, SERIAL_8N1);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  //convert ip Array into String

  ip = String (WiFi.localIP()[0]);
  ip = ip + ".";
  ip = ip + String (WiFi.localIP()[1]);
  ip = ip + ".";
  ip = ip + String (WiFi.localIP()[2]);
  ip = ip + ".";
  ip = ip + String (WiFi.localIP()[3]);

  mqttClient.setClient(espClient);
  mqttClient.setServer( MQTTSERVER, MQTTPORT ); // or local broker
  mqttClient.setCallback(callback);

  previousMillis = millis();
  mqttClient.publish(topic_out, ip.c_str());
  reader.enable(true);

}

void decodeTelegram() {
  if (reader.available()) {
    //-- declaration of DSMRdata must be in
    //-- if-statement so it will be initialized
    //-- in every iteration (don't know how else)
    MyData    DSMRdata;
    String    DSMRerror;
    if (reader.parse(&DSMRdata, &DSMRerror)) {  // Parse succesful, print result
      mEVLT = DSMRdata.energy_delivered_tariff1; //Meter reading Electrics - consumption low tariff
      mEVHT = DSMRdata.energy_delivered_tariff2; //Meter reading Electrics - consumption high tariff
      mEAV = DSMRdata.power_delivered;  //Meter reading Electrics - Actual consumption
      mG = DSMRdata.gas_delivered;   //Meter reading Gas
      Tariff = DSMRdata.electricity_tariff.toInt(); // Current tariff
      Voltage = DSMRdata.voltage_l1; // Voltage
    }
  }
} //Einde 'decodeTelegram()' functie

void sendData() {

  String messageString;

  //send meter data to MQTT
  messageString = String(Voltage);
  messageString.toCharArray(messageBuffer, messageString.length() + 1);
  mqttClient.publish(topic_out_meter_voltage, messageBuffer);
  messageString = String(Tariff);
  messageString.toCharArray(messageBuffer, messageString.length() + 1);
  mqttClient.publish(topic_out_meter_tariff, messageBuffer);
  messageString = String(mEVLT);
  messageString.toCharArray(messageBuffer, messageString.length() + 1);
  mqttClient.publish(topic_out_meter_low, messageBuffer);
  messageString = String(mEVHT);
  messageString.toCharArray(messageBuffer, messageString.length() + 1);
  mqttClient.publish(topic_out_meter_high, messageBuffer);
  messageString = String(mEAV);
  messageString.toCharArray(messageBuffer, messageString.length() + 1);
  mqttClient.publish(topic_out_meter_power, messageBuffer);
  messageString = String(mG);
  messageString.toCharArray(messageBuffer, messageString.length() + 1);
  mqttClient.publish(topic_out_meter_gas, messageBuffer);
  //Reset variables to zero for next run
  Voltage = 0;
  Tariff = 0;
  mEVLT = 0;
  mEVHT = 0;
  mEAV = 0;
  mG = 0;
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    // Attempt to connect
    if (mqttClient.connect(CLIENT_ID)) {
      // Once connected, publish an announcement...
      mqttClient.publish(topic_out, ip.c_str());
      mqttClient.publish(topic_out, "hello world");
      // ... and resubscribe
      mqttClient.subscribe(topic_in);
    } else {
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}



void loop() {
  // Main loop, where we check if we're connected to MQTT...
  if (!mqttClient.connected()) {
    reconnect();
  }

  decodeTelegram();
  sendData();
  delay(5000);
  // and loop.
  mqttClient.loop();
}
