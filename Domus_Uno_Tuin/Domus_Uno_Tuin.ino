/*
          <========Arduino Sketch for Arduino Uno with Ethernet module W5100=========>
          Locatie: Tuin
          Macadres: 00:01:02:03:04:0A
          Pins used:
          2:
          3: DHT-22 sensor
          4: PIR Sensor
          5: Relay 0 (SSR)
          6: Relay 1 (SSR)
          7:
          8: Reset for W5100
          9:
          10: <in gebruik voor W5100>
          11: <in gebruik voor W5100>
          12: <in gebruik voor W5100>
          13: <in gebruik voor W5100>


          incoming topic: domus/tuin/in

          Arduino Uno with W5100 Module used as MQTT client
          It will connect over Wifi to the MQTT broker and controls digital outputs (LED, relays)
          The topics have the format "domus/tuin/uit" for outgoing messages and
          "domus/tuin/in" for incoming messages.
          As the available memory of a UNO  with Ethernetcard is limited,
          I have kept the topics short
          Also, the payloads  are kept short
          The outgoing topics are

          domus/tuin/uit        // Relaisuitgangen: R<relaisnummer><status>
          domus/tuin/uit/rook   // MQ-2 gas & rookmelder, geconverteerd naar 0-100%
          domus/tuin/uit/licht  // LDR-melder: 0=licht, 1=donker
          domus/tuin/uit/temp   // DHT-22 temperatuursensor
          domus/tuin/uit/warmte // DHT-22 gevoelstemperatuur
          domus/tuin/uit/vocht  // DHT-22 luchtvochtigheid
          
          Here, each relay state is reported using the same syntax as the switch command:
          R<relay number><state>

          There is only one incoming topic:
          domus/tuin/in
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

*/
// parameters to tune memory use
//#define BMP280 1 // use BMP280 sensor
#define DHT_present 1 // use DHT sensor
#define DEBUG 1 // Zet debug mode aan

#include <Ethernet.h>           // Ethernet.h library
#include "PubSubClient.h"       //PubSubClient.h Library from Knolleary
//#include <Adafruit_Sensor.h>
//#include <DHT.h>                // Library for DHT-11/22 sensors
//#include <Adafruit_BMP280.h>    // Adafruit BMP280 library

#define BUFFERSIZE 100          // default 100

#if defined(DHT_present)
#include <DHT.h>
#define DHT_PIN 3 // Vul hier de pin in van de DHT11 sensor
DHT dht(DHT_PIN, DHT22);
#endif


#if defined(BMP280)
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; // I2C
bool bmp_present = true;
#endif

// Vul hier de pin in van de PIR
byte PirSensor = 4;
boolean PreviousDetect = false; // Statusvariabele PIR sensor

// Vul hier de naam in waarmee de Arduino zich aanmeldt bij MQTT
#define CLIENT_ID  "domus_tuin"

// Vul hier het interval in waarmee sensorgegevens worden verstuurd op MQTT
#define PUBLISH_DELAY 5000 // that is 3 seconds interval

String hostname = CLIENT_ID;

// Vul hier de MQTT topic in waar deze arduino naar luistert
const char* topic_in = "domus/tuin/in";

// Vul hier de uitgaande MQTT topics in
const char* topic_out = "domus/tuin/uit";
//const char* topic_out_smoke = "domus/tuin/uit/rook";
//const char* topic_out_light = "domus/tuin/uit/licht";
//const char* topic_out_door = "domus/tuin/uit/deur";
const char* topic_out_temp = "domus/tuin/uit/temp";
const char* topic_out_hum = "domus/tuin/uit/vocht";
const char* topic_out_heat = "domus/tuin/uit/warmte";
const char* topic_out_pir = "domus/tuin/uit/pir";

#if defined(BMP280)
const char* topic_out_bmptemp = "domus/tuin/uit/b_temp";
const char* topic_out_pressure = "domus/tuin/uit/druk";
#endif

// Vul hier het aantal gebruikte relais in en de pinnen waaraan ze verbonden zijn
const PROGMEM byte NumberOfRelays = 2;
const PROGMEM byte RelayPins[] = {5, 6};
bool RelayInitialState[] = {LOW, LOW};

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
const PROGMEM uint8_t mac[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x0A};

EthernetClient ethClient;
PubSubClient mqttClient;

long previousMillis;

// General variables
void ShowDebug(String tekst) {
  if (debug) {
    Serial.println(tekst);
  }
}

void setup() {
  for (byte thisPin = 0; thisPin < NumberOfRelays; thisPin++) {
    pinMode(RelayPins[thisPin], OUTPUT);
    digitalWrite(RelayPins[thisPin], RelayInitialState[thisPin]);
  }

#if defined(DHT_present)
  dht.begin();
#endif

  //  pinMode(SmokeSensor, INPUT);
  //  pinMode(PWMoutput, OUTPUT);
  //  pinMode(LightSensor, INPUT);
  
  pinMode(PirSensor, INPUT);

  // setup serial communication

  if (debug) {
    Serial.begin(9600);
    while (!Serial) {};

    ShowDebug(F("MQTT Arduino Domus Tuin"));
    ShowDebug(hostname);
    ShowDebug("");
  }
  
  // setup ethernet communication using DHCP
  if (Ethernet.begin(mac) == 0) {

    ShowDebug(F("Unable to configure Ethernet using DHCP"));
    for (;;);
  }

  ShowDebug(F("Ethernet configured via DHCP"));
  ShowDebug("IP address: ");

  //convert ip Array into String
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
  mqttClient.setServer( "majordomo", 1883); // or local broker
  ShowDebug(F("MQTT client configured"));
  mqttClient.setCallback(callback);
  ShowDebug("");
  ShowDebug(F("Ready to send data"));
  previousMillis = millis();
  //  mqttClient.publish(topic_out, ip.c_str());
#if defined(BMP280)
  if (!bmp.begin()) {
    ShowDebug("Could not find a valid BMP280 sensor, check wiring!");
    bmp_present = false;
  }
#endif
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
      mqttClient.publish(topic_out, "MQTT Arduino Domus Tuin connected");
      // ... and resubscribe
      mqttClient.subscribe(topic_in);
    } else {
      ShowDebug("failed, rc=" + String(mqttClient.state()));
      ShowDebug(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void sendMessage(String m, char* topic) {
  m.toCharArray(messageBuffer, m.length() + 1);
  mqttClient.publish(topic, messageBuffer);
}

void sendData() {

#if defined(DHT_present)
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float hic = dht.computeHeatIndex(t, h, false);

  //  Send Temperature sensor
  sendMessage(String(t), topic_out_temp);

  //  Send Humidity sensor
  sendMessage(String(h), topic_out_hum);

  //  Send Heat index sensor
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
  // Send status of relays
  for (byte thisPin = 0; thisPin < NumberOfRelays; thisPin++) {
    report_state(thisPin);
  }
}

void report_state(byte outputport)
{
  String messageString = "R" + String(outputport) + String(digitalRead(RelayPins[outputport]));
  sendMessage(messageString, topic_out);
}

void callback(char* topic, byte * payload, byte length) {
  char msgBuffer[20];
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
    ShowDebug("Relay command");

    RelayPort = strPayload[1] - 48;
    if (RelayPort > 16) RelayPort -= 3;
    RelayValue = strPayload[2] - 48;

    ShowDebug(String(RelayPort));
    ShowDebug(String(RelayValue));
    ShowDebug(String(HIGH));

    if (RelayValue == 40) {
      ShowDebug("Toggling relay...");
      digitalWrite(RelayPins[RelayPort], !digitalRead(RelayPins[RelayPort]));
    } else {
      digitalWrite(RelayPins[RelayPort], RelayValue);
    }
    report_state(RelayPort);
  } else if (strPayload == "IP")  {

    // 'Show IP' commando
    mqttClient.publish(topic_out, ip.c_str());// publish IP nr
  }
  else if (strPayload == "AON") {

    // Alle relais aan
    for (byte i = 0 ; i < NumberOfRelays; i++) {
      digitalWrite(RelayPins[i], 1);
      report_state(i);
    }
  }

  else if (strPayload == "AOF") {

    // Alle relais uit
    for (byte i = 0 ; i < NumberOfRelays; i++) {
      digitalWrite(RelayPins[i], 0);
      report_state(i);
    }
  }
  else if (strPayload == "STAT") {

    // Status van alle sensors and relais
    sendData();
  }

  else {

    // Onbekend commando
    ShowDebug("Unknown value");
    mqttClient.publish(topic_out, "Unknown command");
  }
}

void loop() {
  // Main loop, where we check if we're connected to MQTT...
  if (!mqttClient.connected()) {
    ShowDebug("Not Connected!");
    reconnect();
  }

  // ... then send all relay stats when we've just started up....
  if (startsend) {
    for (byte thisPin = 0; thisPin < NumberOfRelays; thisPin++) {
      report_state(thisPin);
    }
    startsend = false;
  }

  // ...see if it's time to send new data, ....
  if (millis() - previousMillis > PUBLISH_DELAY) {
    previousMillis = millis();
    sendData();
  }

  // ...read out the PIR sensor...
  if (digitalRead(PirSensor) == HIGH) {
    if (!PreviousDetect) {
      ShowDebug("Detecting movement.");
      sendMessage("on", topic_out_pir);
      PreviousDetect = true;
    }
  }
  else {
    if (PreviousDetect) {
      sendMessage("off", topic_out_pir);
    }
    PreviousDetect = false;
  }

  // and loop.
  mqttClient.loop();
}
