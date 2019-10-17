/*
          <========Arduino Sketch for Arduino Mega with Ethernet shield W5100=========>
          Location: Meterkast
          Macadres: 00:01:02:03:04:0B
          Pins used:
          2: PWM voor LEDs
          3: Temperatuursensor (DHT of 18b20)
          4: <in gebruik voor W5100>
          5: Relay 0 (not connected)
          6: Relay 1 (not connected)
          7: Relay 2/PulseRelay 1 (Pulse, Voordeuropener)
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

          incoming topic: domus/mk/in

          Arduino MEGA with W5100 Ethernetshield used as MQTT client
          It will connect over Wifi to the MQTT broker and controls digital outputs (LED, relays)
          The topics have the format "domus/hk/uit" for outgoing messages and
          "domus/hk/in" for incoming messages.
          As the available memory of a UNO  with Ethernetcard is limited,
          I have kept the topics short
          Also, the payloads  are kept short

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
          C<cover number>[<pulse length>] - close cover
          O<cover number>[<pulse length>] - open cover
          S<cover number> - stops cover movement

          On Startup, the Client publishes the IP number

          Adapted 4-2-2018 by Peter Mansvelder:

          removed Temp/Humidity, added multiple relays for MQTT house control
          I used the following ports:

          Uno: pins 4,10,11,12,13 in use
          Mega: 4,10,50,51,52,53 in use

          3,5,6,7,8,9,A0(14),A1(15),A2(16),A3(17), using those not used by ethernet shield (4, 10, 11, 12, 13) and other
          ports (0, 1 used by serial interface).

          Adapted 24-02-2018 by Peter Mansvelder:
          - added sensors for light and smoke (MQ-2), reporting on output topics
          - added alarm function for smoke with buzzer
          - added pulse relay

          Adapted 22-07-2018 by Peter Mansvelder
          - added smart meter P1 input (later removed)

          Adapted 4-11-2018 by Peter Mansvelder
          - added cover control

*/

#include "secrets.h"

// parameters to tune memory use
//#define BMP280 1 // use BMP280 sensor
//#define DHT_present 1 // use DHT sensor
//#define DS18B20_present 1 // Use OneWire temperature sensor

// setup for oled display on i2c
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000
};

// setup for ethernet
#include <Ethernet.h>// Ethernet.h library
#include "PubSubClient.h" //PubSubClient.h Library from Knolleary#define MQTT_SERVER "majordomo"

#if defined(DHT_present)
#include <DHT.h>
#include <Adafruit_Sensor.h>
#define DHT_PIN 3 // Vul hier de pin in van de DHT11 sensor
DHT dht(DHT_PIN, DHT22);
#endif

#if defined(DS18B20_present)
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 3
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
#endif

// Vul hier de data in van de PIRs
byte NumberOfPirs = 3;
int PirSensors[] = {28, 29, 19};
int PreviousDetects[] = {false, false, false}; // Statusvariabele PIR sensor

// Vul hier de gegevens in van de motorsturing voor de screens:
// 2 relais per motor: 1 x richting, 1 x motorpuls
// hiervoor gebruik ik de pulserelais en de normale relais
// de waarden zijn de indices op de onderstaande 'RelayPins' en 'PulseRelayPins' arrays
byte NumberOfCovers = 2;
byte CoverDir[] = {6, 8}; // relay numbers for direction
byte CoverPulse[] = {2, 3}; // relay numbers for motor pulses
byte CoverState[] = {false, false}; // false = open, true = closed
#define COVERDELAYTIME 30000 // time to wait for full open or close

// Vul hier de naam in waarmee de Arduino zich aanmeldt bij MQTT
#define CLIENT_ID  "domus_meterkast_screens"

// Vul hier het interval in waarmee sensorgegevens worden verstuurd op MQTT
#define PUBLISH_DELAY 1000 // that is 5 seconds interval
#define DEBOUNCE_DELAY 150
#define LONGPRESS_TIME 450

String hostname = CLIENT_ID;

// Vul hier de MQTT topic in waar deze arduino naar luistert
const char* topic_in = "domus/mk/in";

// Vul hier de uitgaande MQTT topics in
const char* topic_out = "domus/mk/uit";               // Relaisuitgangen: R<relaisnummer><status>
const char* topic_out_smoke = "domus/mk/uit/rook";    // MQ-2 gas & rookmelder, geconverteerd naar 0-100%
const char* topic_out_light = "domus/mk/uit/licht";   // LDR-melder: 0=licht, 1=donker
const char* topic_out_pulse = "domus/mk/uit/deur";    // Pulserelais t.b.v. deuropener
#if defined(DHT_present)
const char* topic_out_temp = "domus/mk/uit/temp";     // DHT-22 temperatuursensor
const char* topic_out_hum = "domus/mk/uit/vocht";     // DHT-22 luchtvochtigheid
const char* topic_out_heat = "domus/mk/uit/warmte";   // DHT-22 gevoelstemperatuur
#endif

#if defined(DS18B20_present)
const char* topic_out_temp = "domus/mk/uit/temp";     // 18b20 1-wiretemperatuursensor
#endif

const char* topic_out_pir = "domus/mk/uit/pir";       // PIR sensors
const char* topic_out_screen = "domus/mk/uit/screen"; // Screens (zonwering)

// Vul hier het aantal gebruikte relais in en de pinnen waaraan ze verbonden zijn
int NumberOfRelays = 9;
int RelayPins[] = {5, 6, 7, 30, 31, 22, 23, 24, 25};
bool RelayInitialState[] = {HIGH, HIGH, HIGH, LOW, LOW, HIGH, HIGH, HIGH, HIGH};

// Vul hier het aantal pulsrelais in
int NumberOfPulseRelays = 4; // 0 = haldeur, 1 = voordeur, 2 = screen keuken, 3 = screen huiskamer
// Vul hier de pins in van het pulserelais.
int PulseRelayPins[] = {8, 7, 22, 24};
long PulseActivityTimes[] = {0, 0, 0, 0};
// Vul hier de default status in van het pulsrelais (sommige relais vereisen een 0, andere een 1 om te activeren)
// gebruikt 5V YwRobot relay board vereist een 0, 12 volt insteekrelais een 1, SSR relais een 1.
bool PulseRelayInitialStates[] = {HIGH, HIGH, HIGH, HIGH};
// Vul hier de pulsetijden in voor de pulserelais
long int PulseRelayTimes[] = {2000, 250, COVERDELAYTIME, COVERDELAYTIME};

// Vul hier het aantal knoppen in en de pinnen waaraan ze verbonden zijn
int NumberOfButtons = 3;
int ButtonPins[] = {11, 12, 9};
static byte lastButtonStates[] = {0, 0, 0};
long lastActivityTimes[] = {0, 0, 0};
long LongPressActive[] = {0, 0, 0};

// Vul hier de pin in van de rooksensor
int SmokeSensor = A0;

// Vul hier de pwm outputpin in voor de Ledverlichting van de knoppen
int PWMoutput = 2; // Uno: 3, 5, 6, 9, 10, and 11, Mega: 2 - 13 and 44 - 46

// Vul hier de pin in van de lichtsensor
// int LightSensor = 2;

char messageBuffer[100];
char topicBuffer[100];
String ip = "";
bool startsend = HIGH;// flag for sending at startup
bool debug = true;
int lichtstatus; //contains LDR reading

// Vul hier het macadres in
uint8_t mac[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x0B};

EthernetClient ethClient;
PubSubClient mqttClient;

long previousMillis;

void(* resetFunc) (void) = 0; //declare reset function @ address 0

// General variables
void ShowDebug(String tekst) {
  if (debug) {
    Serial.println(tekst);
  }
}

void displayMessage(String text) {
  display.fillRect(0, 24, 131, 31, BLACK);
  display.setCursor(0, 24);
  display.println(text);
  display.display();
}

void displayTime() {
  display.fillRect(0, 0, 131, 7, BLACK);
  display.setCursor(0, 0);
  display.print("Time:");
  display.println(TimeToString(millis() / 1000));
  display.display();
}

void setup() {

  if (debug) {
    Serial.begin(9600);   // setup serial communication
    while (!Serial) {};

    ShowDebug(F("MQTT Arduino Domus"));
    ShowDebug(hostname);
    ShowDebug("");
  }

  // setup oled display
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(1000);
  display.clearDisplay();
  display.display();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);
  display.println(F("meterkast_screens"));

  displayMessage("Setup relay pins");

  // setup relay pins
  for (int thisPin = 0; thisPin < NumberOfRelays; thisPin++) {
    pinMode(RelayPins[thisPin], OUTPUT);
    ShowDebug("Enabling relay pin " + String(RelayPins[thisPin]));
    digitalWrite(RelayPins[thisPin], RelayInitialState[thisPin]);
  }

  // set up pulse relay pins.
  displayMessage("Setup pulse relay pins");

  for (int thisPin = 0; thisPin < NumberOfPulseRelays; thisPin++) {
    pinMode(PulseRelayPins[thisPin], OUTPUT);
    ShowDebug("Enabling pulse relay pin " + String(PulseRelayPins[thisPin]));
    digitalWrite(PulseRelayPins[thisPin], PulseRelayInitialStates[thisPin]);
  }
  // set up buttons

  displayMessage("Setup button pins");

  for (int thisButton = 0; thisButton < NumberOfButtons; thisButton++) {
    ShowDebug("Enabling button pin " + String(ButtonPins[thisButton]));
    pinMode(ButtonPins[thisButton], INPUT_PULLUP);
  }

#if defined(DHT_present)
  dht.begin();
#endif

  //  pinMode(SmokeSensor, INPUT);
  //  pinMode(PWMoutput, OUTPUT);
  //  pinMode(LightSensor, INPUT);

  // set up pirs
  displayMessage("Setup pir pins");

  for (byte pirid = 0; pirid < NumberOfPirs; pirid++) {
    ShowDebug("Enabling pir pin " + String(PirSensors[pirid]));
    pinMode(PirSensors[pirid], INPUT_PULLUP);
  }

  //   setup ethernet communication using DHCP
  displayMessage("Setup ethernet");

  if (Ethernet.begin(mac) == 0) {

    ShowDebug(F("Unable to configure Ethernet using DHCP"));
    delay(1000);
    resetFunc();
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

  // setup mqtt client
  displayMessage("Setup MQTT");
  mqttClient.setClient(ethClient);
  mqttClient.setServer(MQTTSERVER, 1883); // or local broker
  ShowDebug(F("MQTT client configured"));
  mqttClient.setCallback(callback);
  ShowDebug(F("Ready to send data"));
  previousMillis = millis();

  displayMessage("Setup done.");
} // setup

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

void ProcessPulseRelays(int PulseRelayId) {
  // Process the timers of the pulse relays and see if we have to close them.
  if (((millis() - PulseActivityTimes[PulseRelayId]) > PulseRelayTimes[PulseRelayId]) && digitalRead(PulseRelayPins[PulseRelayId]) == !PulseRelayInitialStates[PulseRelayId])
  {
    ShowDebug("Disabling pulse relay" + String(PulseRelayId) + ".");
    ShowDebug(String(PulseActivityTimes[PulseRelayId]));
    String messageString = "P" + String(PulseRelayId) + "0";
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out_pulse, messageBuffer);
    digitalWrite(PulseRelayPins[PulseRelayId], PulseRelayInitialStates[PulseRelayId]);
  }
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
      ShowDebug(" reset in 5 seconds");
      // Wait 5 seconds before resetting
      delay(5000);
      resetFunc();
    }
  }
}

// t is time in seconds = millis()/1000;
char * TimeToString(unsigned long t)
{
  static char str[12];
  long h = t / 3600;
  t = t % 3600;
  int m = t / 60;
  int s = t % 60;
  sprintf(str, "%04ld:%02d:%02d", h, m, s);
  return str;
}

void sendData() {
  //  int smoke = analogRead(SmokeSensor);
  //  bool light = digitalRead(LightSensor);
  String messageString;

#if defined(DHT_present)
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float hic = dht.computeHeatIndex(t, h, false);

  // Send Temperature sensor
  messageString = String(t);
  messageString.toCharArray(messageBuffer, messageString.length() + 1);
  mqttClient.publish(topic_out_temp, messageBuffer);

  // Send Humidity sensor
  messageString = String(h);
  messageString.toCharArray(messageBuffer, messageString.length() + 1);
  mqttClient.publish(topic_out_hum, messageBuffer);

  // Send Heat index sensor
  messageString = String(hic);
  messageString.toCharArray(messageBuffer, messageString.length() + 1);
  mqttClient.publish(topic_out_heat, messageBuffer);
#endif

#if defined(DS18B20_present)
  float t2 = sensors.getTempCByIndex(0);
  //  Send Temperature sensor
  ShowDebug("Temperatuur: " + String(t2));
  sendMessage(String(t2), topic_out_temp);
  sensors.requestTemperatures();
#endif

  // Send smoke sensor
  //  messageString = String(map(smoke, 0, 1023, 0, 100));
  //  messageString.toCharArray(messageBuffer, messageString.length() + 1);
  //  mqttClient.publish(topic_out_smoke, messageBuffer);

  // Send light sensor
  //  messageString = String(light);
  //  messageString.toCharArray(messageBuffer, messageString.length() + 1);
  //  mqttClient.publish(topic_out_light, messageBuffer);

  // Send status of relays
  display.fillRect(0, 10, 131, 14, BLACK);
  display.setCursor(0, 12);
  display.print("R:");
  for (int thisPin = 0; thisPin < NumberOfRelays; thisPin++) {
    report_state(thisPin);
  }
  display.print("/C:");
  // send status of screens
  for (int cover = 1; cover <= NumberOfCovers; cover++) {
    report_state_cover(cover);
  }
  display.display();
  displayTime();
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

void report_state(int outputport)
{
  ShowDebug("R" + String(outputport) + String(digitalRead(RelayPins[outputport])));
  String messageString = "R" + String(outputport) + String(digitalRead(RelayPins[outputport]));
  messageString.toCharArray(messageBuffer, messageString.length() + 1);
  mqttClient.publish(topic_out, messageBuffer);
  display.print(String(digitalRead(RelayPins[outputport])));
}

void report_state_cover(int CoverPort)
{
  // report state of screen
  String messageString = "";
  if (CoverState[CoverPort - 1])
  {
    messageString = "C" + String(CoverPort);
  }
  else
  {
    messageString = "O" + String(CoverPort);
  }
  ShowDebug("Cover" + String(CoverPort) + CoverState[CoverPort - 1]);
  messageString.toCharArray(messageBuffer, messageString.length() + 1);
  mqttClient.publish(topic_out_screen, messageBuffer);
  display.print(messageString);
}

void callback(char* topic, byte * payload, unsigned int length) {
  // this function listens on a topic, and takes the required action
  // implemented functions;
  // IP to publish IP address
  // AON to set all relays to on state
  // AOF to set all relays to off state
  // STAT to show all relay states
  // RXY to set Relay X to state Y
  // PX(Y) to enable pulse relay X (reports back as state of relay, so PXY)
  // LXXX to set PWM output to leds to value XXX
  // *** experimental for screens ***
  // implement OPEN, CLOSE, STOP for HA covers
  // OX to open cover X
  // CX to close cover X
  // SX to stop motion on cover X
  // *** experimental end ***
  char msgBuffer[20];
  // I am only using one ascii character as command, so do not need to take an entire word as payload
  // However, if you want to send full word commands, uncomment the next line and use for string comparison
  payload[length] = '\0'; // terminate string with 0
  String strPayload = String((char*)payload);  // convert to string
  ShowDebug(strPayload);
  ShowDebug("Message arrived");
  ShowDebug(topic);
  ShowDebug(strPayload);
  displayMessage("Incoming:" + strPayload);

  int RelayPort;
  int RelayValue;

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
    for (int i = 0 ; i < NumberOfRelays; i++) {
      digitalWrite(RelayPins[i], 1);
      report_state(i);
    }
  }
  else if (strPayload == "AOF") {
    // Alle relais uit
    for (int i = 0 ; i < NumberOfRelays; i++) {
      digitalWrite(RelayPins[i], 0);
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
  else if (strPayload[0] == 'P') {  // PULSE RELAY
    int PulseRelayPort = strPayload[1] - 48;
    if (strPayload[2] == '1') {  // Pulserelay on
      ShowDebug("Enabling pulse relay " + String(PulseRelayPort) + ".");
      digitalWrite(PulseRelayPins[PulseRelayPort], !PulseRelayInitialStates[PulseRelayPort]);
      String messageString = "P" + String(PulseRelayPort) + "1";
      messageString.toCharArray(messageBuffer, messageString.length() + 1);
      mqttClient.publish(topic_out_pulse, messageBuffer);
      PulseActivityTimes[PulseRelayPort] = millis();
    }
    else { // Pulserelay forced off
      ShowDebug("Disabling pulse relay " + String(PulseRelayPort) + ".");
      String messageString = "P" + String(PulseRelayPort) + "0";
      messageString.toCharArray(messageBuffer, messageString.length() + 1);
      mqttClient.publish(topic_out_pulse, messageBuffer);
      digitalWrite(PulseRelayPins[PulseRelayPort], PulseRelayInitialStates[PulseRelayPort]);
    }
  }
  else if (strPayload[0] == 'L') {  // LED PWM
    analogWrite(PWMoutput, strPayload.substring(1).toInt());
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
        CoverState[CoverPort - 1] = false; // false is open
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
        CoverState[CoverPort - 1] = true;
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

void loop() {
  // Main loop, where we check if we're connected to MQTT...
  if (!mqttClient.connected()) {
    ShowDebug("Not Connected!");
    reconnect();
  }

  // ... then send all relay stats when we've just started up....
  if (startsend) {
    for (int thisPin = 0; thisPin < NumberOfRelays; thisPin++) {
      report_state(thisPin);
    }
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
} // loop
