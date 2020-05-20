/*
          <========Arduino Sketch for Arduino Uno Wifi=========>
          Locatie: Tuin

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

          Arduino Uno Wifi rev2 used as MQTT client
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
//#define BMP280 0 // use BMP280 sensor
//#define DHT_present 1 // use DHT sensor
//#define MQ_present 1 // MQ-x gas sensor
#define MS_present // YL-69 moisture sensor
#define DEBUG 1 // Zet debug mode aan

//#include <Ethernet.h>           // Ethernet.h library
#include <WiFiNINA.h>
#include <utility/wifi_drv.h>

#include "secrets.h"
#include "PubSubClient.h"       //PubSubClient.h Library from Knolleary
//#include <Adafruit_Sensor.h>

#define BUFFERSIZE 100          // default 100

#if defined(DHT_present)
#include <DHT.h>
#define DHT_PIN 3 // Vul hier de pin in van de DHT11 sensor
DHT dht(DHT_PIN, DHT22);
#endif

#if defined(MQ_present)
#define MQ_PIN A3 // Vul hier de pin in van de DHT11 sensor
#endif

#if defined(MS_present)
#define MS_PIN A0 // Vul hier de pin in van de moisture sensor
#define MS_ENABLE 8 // vul hier de enable pin in
#define SAMPLESIZE 10 // aantal keren meten
const char* topic_out_moisture = "domus/tuin/uit/moisture";
#endif

#if defined(BMP280)
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; // I2C
bool bmp_present = true;
#endif

// Vul hier de naam in waarmee de Arduino zich aanmeldt bij MQTT
#define CLIENT_ID  "domus_tuin"

// Vul hier het interval in waarmee sensorgegevens worden verstuurd op MQTT
#define PUBLISH_DELAY 5000 // that is 5 seconds interval

String hostname = CLIENT_ID;

// Vul hier de data in van de PIRs
byte NumberOfPirs = 0;
int PirSensors[] = {1};
int PreviousDetects[] = {false}; // Statusvariabele PIR sensor

// Vul hier de MQTT topic in waar deze arduino naar luistert
const char* topic_in = "domus/tuin/in";

// Vul hier de uitgaande MQTT topics in
const char* topic_out = "domus/tuin/uit";
//const char* topic_out_smoke = "domus/tuin/uit/rook";
//const char* topic_out_light = "domus/tuin/uit/licht";
//const char* topic_out_door = "domus/tuin/uit/deur";

#if defined(DHT_present)
const char* topic_out_temp = "domus/tuin/uit/temp";
const char* topic_out_hum = "domus/tuin/uit/vocht";
const char* topic_out_heat = "domus/tuin/uit/warmte";
#endif

const char* topic_out_pir = "domus/tuin/uit/pir";

#if defined(MQ_present)
const char* topic_out_gas = "domus/tuin/uit/gas";
#endif

#if defined(BMP280)
const char* topic_out_bmptemp = "domus/tuin/uit/b_temp";
const char* topic_out_pressure = "domus/tuin/uit/druk";
#endif

// Vul hier het aantal gebruikte relais in en de pinnen waaraan ze verbonden zijn
const byte NumberOfRelays = 0;
const byte RelayPins[] = {10, 11, 12, 13};
PinStatus RelayInitialState[] = {LOW, LOW, LOW, LOW};

char messageBuffer[BUFFERSIZE];
char topicBuffer[BUFFERSIZE];
String ip = "";
bool startsend = HIGH;// flag for sending at startup
#if defined (DEBUG)
bool debug = true;
#else
bool debug = false;
#endif

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
/////// Wifi Settings ///////
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

int status = WL_IDLE_STATUS;      // the Wifi radio's status

WiFiClient wifi;

PubSubClient mqttClient(wifi);

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
int ButtonPins[] = {6, 7, 8, 9};
static byte lastButtonStates[] = {0, 0, 0, 0};
long lastActivityTimes[] = {0, 0, 0, 0};
long LongPressActive[] = {0, 0, 0, 0};

#define DEBOUNCE_DELAY 150
#define LONGPRESS_TIME 450

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

void reconnect() {
  // Loop until we're reconnected
  set_rgb_led(64, 0, 0);  // RED
  status = WiFi.begin(ssid, pass);
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
  }
  set_rgb_led(0, 64, 0); // GREEN
  while (!mqttClient.connected()) {
    ShowDebug("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(CLIENT_ID)) {
      ShowDebug("connected");
      set_rgb_led(0, 0, 64);  // BLUE
      // Once connected, publish an announcement...
      mqttClient.publish(topic_out, ip.c_str());
      mqttClient.publish(topic_out, "MQTT Arduino Domus Tuin connected");
      // ... and resubscribe
      mqttClient.subscribe(topic_in);
    } else {
      set_rgb_led(64, 0, 0);  // RED
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


  ShowDebug("sendData function entered.");
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

#if defined(MQ_present)
  ShowDebug("Sending moisture data...");
  analogWrite(MQ_PIN, HIGH);
  //  delay(50);
  byte g = analogRead(MQ_PIN);
  sendMessage(String(g), topic_out_gas);
#endif

#if defined(MS_present)
  digitalWrite(MS_ENABLE, HIGH);
  float g = 0;
  delay(50);
  for (int i = 0; i < SAMPLESIZE; i++) {
    g += analogRead(MS_PIN);
  }
  g /= SAMPLESIZE;
  ShowDebug("sending moisture data:");
  sendMessage(String(map(g,0,1023,100,0)), topic_out_moisture);
  digitalWrite(MS_ENABLE, LOW);
#endif

  // Send status of relays
  for (byte thisPin = 0; thisPin < NumberOfRelays; thisPin++) {
    report_state(thisPin);
  }
}

void set_rgb_led(byte red, byte green, byte blue)
{
  WiFiDrv::analogWrite(25, red);  // for configurable brightness
  WiFiDrv::analogWrite(26, green);  // for configurable brightness
  WiFiDrv::analogWrite(27, blue);  // for configurable brightness
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

    if (RelayValue == 40) {
      ShowDebug("Toggling relaypin " + String (RelayPins[RelayPort]));
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
    report_state(RelayPort);
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
      report_state(i);
    }
  }

  else if (strPayload == "AOF") {

    // Alle relais uit
    for (byte i = 0 ; i < NumberOfRelays; i++) {
      digitalWrite(RelayPins[i], RelayInitialState[i]);
      report_state(i);
    }
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
      ShowDebug("Detecting movement on pir " + String(pirid) + ".");
      String messageString = "pir" + String(pirid) + "on";
      messageString.toCharArray(messageBuffer, messageString.length() + 1);
      mqttClient.publish(topic_out_pir, messageBuffer);
      PreviousDetects[pirid] = true;
    }
  }
  else {
    if (PreviousDetects[pirid]) {
      ShowDebug("No more movement on pir " + String(pirid) + ".");
      String messageString = "pir" + String(pirid) + "off";
      messageString.toCharArray(messageBuffer, messageString.length() + 1);
      mqttClient.publish(topic_out_pir, messageBuffer);
    }
    PreviousDetects[pirid] = false;
  }
}

void setup() {

  WiFiDrv::pinMode(25, OUTPUT);  //RED
  WiFiDrv::pinMode(26, OUTPUT);  //GREEN
  WiFiDrv::pinMode(27, OUTPUT);  //BLUE
  set_rgb_led(64, 64, 64); // Set Wifi Status LED to WHITE

  if (debug) {
    Serial.begin(9600);
    ShowDebug(F("MQTT Arduino Domus Test"));
    ShowDebug(hostname);
    ShowDebug("");
  }
  ShowDebug("Starting up...");
  // attempt to connect to Wifi network:

  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
  }
  set_rgb_led(0, 64, 0); // GREEN

  if (debug) {
    // you're connected now, so print out the data:
    Serial.print("You're connected to the network IP = ");
    IPAddress ip = WiFi.localIP();
    Serial.println(ip);
  }

  for (byte thisPin = 0; thisPin < NumberOfRelays; thisPin++) {
    pinMode(RelayPins[thisPin], OUTPUT);
    digitalWrite(RelayPins[thisPin], RelayInitialState[thisPin]);
  }

  for (int thisButton = 0; thisButton < NumberOfButtons; thisButton++) {
    ShowDebug("Enabling button pin " + String(ButtonPins[thisButton]));
    pinMode(ButtonPins[thisButton], INPUT_PULLUP);
  }

#if defined(DHT_present)
  dht.begin();
#endif

#if defined(MS_present)
  analogReference(VDD);
  pinMode(MS_ENABLE, OUTPUT); // vul hier de enable pin in
#endif

#if defined(MQ_present)
  ShowDebug("Setting up pin " + String(MQ_PIN) + " as MQ gas sensor");
  analogWrite(MQ_PIN, HIGH);
  ShowDebug("Heating up for 60 seconds...");
  delay(60000);
  // now reducing the heating power: turn the heater to approx 1,4V
  ShowDebug("Reducing voltage to 1,4V and heat for 90 seconds...");
  analogWrite(MQ_PIN, 71.4);// 255x1400/5000
  delay(90000);
  ShowDebug("MQ-7 sensor setup done");
#endif

  //  pinMode(SmokeSensor, INPUT);
  //  pinMode(PWMoutput, OUTPUT);
  //  pinMode(LightSensor, INPUT);

  for (byte pirid = 0; pirid < NumberOfPirs; pirid++) {
    ShowDebug("Enabling pir pin " + String(PirSensors[pirid]));
    pinMode(PirSensors[pirid], INPUT);
  }

  // setup serial communication

  //convert ip Array into String
  ip = String (WiFi.localIP()[0]);
  ip = ip + ".";
  ip = ip + String (WiFi.localIP()[1]);
  ip = ip + ".";
  ip = ip + String (WiFi.localIP()[2]);
  ip = ip + ".";
  ip = ip + String (WiFi.localIP()[3]);

  // setup mqtt client
  mqttClient.setServer( MQTTSERVER, 1883); // or local broker
  ShowDebug(F("MQTT client configured"));
  set_rgb_led(32, 64, 0);  // Set status LED
  mqttClient.setCallback(callback);
  ShowDebug("");
  ShowDebug(F("Ready to send data"));
  previousMillis = millis();
  mqttClient.publish(topic_out, ip.c_str());
#if defined(BMP280)
  if (!bmp.begin()) {
    ShowDebug("Could not find a valid BMP280 sensor, check wiring!");
    bmp_present = false;
  }
#endif
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

  // ...read out the PIR sensors...
  for (int id = 0; id < NumberOfPirs; id++) {
    check_pir(id);
  }

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
