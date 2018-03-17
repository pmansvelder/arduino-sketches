/*
          <========Arduino Sketch for Arduino Mega with Ethernet shield W5100=========>
          Locatie: Meterkast
          Macadres: 00:01:02:03:04:08
          Pins used:
          2: PWM voor LEDs
          3
          5: Relay 0
          6: Relay 1 (Plafondlamp huiskamer)
          7: Relay 2
          8: Relay 3 (Pulse, Deuropener)
          9:  Button 2 (keuken)
          11: Button 0 (huiskamer)
          12: Button 1 (huiskamer)
          13:

          incoming topic: domus/mk/in

          Arduino UNO with W5100 Ethernetshield or W5100 Ethernet module, used as MQTT client
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

*/

#include <Ethernet.h>// Ethernet.h library
#include "PubSubClient.h" //PubSubClient.h Library from Knolleary
#include <Adafruit_Sensor.h>
#include <DHT.h>

// Vul hier de pin in van de DHT11 sensor
#define DHT_PIN 3
// DHT dht(DHT_PIN, DHT11);

// Vul hier de naam in waarmee de Arduino zich aanmeldt bij MQTT
#define CLIENT_ID  "domus_meterkast"

// Vul hier het interval in waarmee sensorgegevens worden verstuurd op MQTT
#define PUBLISH_DELAY 3000 // that is 3 seconds interval
#define DEBOUNCE_DELAY 150
#define LONGPRESS_TIME 450

// Vul hier de tijd in voor het pulserelais
#define PULSE_RELAY_TIME 2500

String hostname = CLIENT_ID;

// Vul hier de MQTT topic in waar deze arduino naar luistert
const char* topic_in = "domus/mk/in";

// Vul hier de uitgaande MQTT topics in
const char* topic_out = "domus/mk/uit";
const char* topic_out_smoke = "domus/mk/uit/rook";
const char* topic_out_light = "domus/mk/uit/licht";
const char* topic_out_door = "domus/mk/uit/deur";
const char* topic_out_temp = "domus/mk/uit/temp";
const char* topic_out_hum = "domus/mk/uit/vocht";
const char* topic_out_heat = "domus/mk/uit/warmte";

// Vul hier het aantal gebruikte relais in en de pinnen waaraan ze verbonden zijn
int NumberOfRelays = 4;
int RelayPins[] = {5, 6, 7, 30};
bool RelayInitialState[] = {HIGH, HIGH, HIGH, LOW};

// Vul hier de pin in van het pulserelais.
int PulseRelayPin = 8;
bool PulseRelayInitialState = HIGH;
long PulseActivityTime = 0;

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
bool statusKD = HIGH;
bool statusBD = HIGH;
bool statusGD = HIGH;
bool relaystate1 = LOW;
bool pir = LOW;
bool startsend = HIGH;// flag for sending at startup
bool debug = true;
int lichtstatus; //contains LDR reading

// Vul hier het macadres in
uint8_t mac[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x08};

EthernetClient ethClient;
PubSubClient mqttClient;

long previousMillis;

void ShowDebug(String tekst) {
  if (debug) {
    Serial.println(tekst);
  }
}

void setup() {
  for (int thisPin = 0; thisPin < NumberOfRelays; thisPin++) {
    pinMode(RelayPins[thisPin], OUTPUT);
    digitalWrite(RelayPins[thisPin], RelayInitialState[thisPin]);
  }

  for (int thisButton = 0; thisButton < NumberOfButtons; thisButton++) {
    pinMode(ButtonPins[thisButton], INPUT_PULLUP);
  }

  // dht.begin();

  //  pinMode(SmokeSensor, INPUT);
  //  pinMode(PWMoutput, OUTPUT);
  //  pinMode(LightSensor, INPUT);
  pinMode(PulseRelayPin, OUTPUT);
  digitalWrite(PulseRelayPin, PulseRelayInitialState);

  // setup serial communication

  if (debug) {
    Serial.begin(9600);
    while (!Serial) {};

    ShowDebug(F("MQTT Arduino Domus"));
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
  //  mqttClient.setServer( "192.168.178.37", 1883); // or local broker
  mqttClient.setServer( "majordomo", 1883); // or local broker
  ShowDebug(F("MQTT client configured"));
  mqttClient.setCallback(callback);
  ShowDebug("");
  ShowDebug(F("Ready to send data"));
  previousMillis = millis();
  //  mqttClient.publish(topic_out, ip.c_str());
}

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
    else if ((millis() - lastActivityTimes[buttonId] > LONGPRESS_TIME) && (LongPressActive[buttonId] == false))// Button long press
    {
      LongPressActive[buttonId] = true;
      ShowDebug( "Button" + String(buttonId) + " long pressed" );
      String messageString = "Button" + String(buttonId) + "_long";
      messageString.toCharArray(messageBuffer, messageString.length() + 1);
      mqttClient.publish(topic_out, messageBuffer);
    }
    lastButtonStates[buttonId] = 1;
  }
  else {
    if (lastButtonStates[buttonId] == true) {
      if (LongPressActive[buttonId] == true) {
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
      lastButtonStates[buttonId] = false;
    }
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
      mqttClient.publish(topic_out, "hello world");
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

void sendData() {
  //  int smoke = analogRead(SmokeSensor);
  //  bool light = digitalRead(LightSensor);
  String messageString;

  //  float h = dht.readHumidity();
  //  float t = dht.readTemperature();
  //  float hic = dht.computeHeatIndex(t, h, false);

  // Send Temperature sensor
  //  messageString = String(t);
  //  messageString.toCharArray(messageBuffer, messageString.length() + 1);
  //  mqttClient.publish(topic_out_temp, messageBuffer);

  // Send Humidity sensor
  //  messageString = String(h);
  //  messageString.toCharArray(messageBuffer, messageString.length() + 1);
  //  mqttClient.publish(topic_out_hum, messageBuffer);

  // Send Heat index sensor
  //  messageString = String(hic);
  //  messageString.toCharArray(messageBuffer, messageString.length() + 1);
  //  mqttClient.publish(topic_out_heat, messageBuffer);

  // Send smoke sensor
  //  messageString = String(map(smoke, 0, 1023, 0, 100));
  //  messageString.toCharArray(messageBuffer, messageString.length() + 1);
  //  mqttClient.publish(topic_out_smoke, messageBuffer);

  // Send light sensor
  //  messageString = String(light);
  //  messageString.toCharArray(messageBuffer, messageString.length() + 1);
  //  mqttClient.publish(topic_out_light, messageBuffer);
}

void report_state(int outputport)
{
  String messageString = "R" + String(outputport) + String(digitalRead(RelayPins[outputport]));
  messageString.toCharArray(messageBuffer, messageString.length() + 1);
  mqttClient.publish(topic_out, messageBuffer);
}

void callback(char* topic, byte * payload, unsigned int length) {
  char msgBuffer[20];
  // I am only using one ascii character as command, so do not need to take an entire word as payload
  // However, if you want to send full word commands, uncomment the next line and use for string comparison
  payload[length] = '\0'; // terminate string with 0
  String strPayload = String((char*)payload);  // convert to string
  ShowDebug(strPayload);
  ShowDebug("Message arrived");
  ShowDebug(topic);
  ShowDebug(strPayload);

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

    // Status van alle relais
    for (int thisPin = 0; thisPin < NumberOfRelays; thisPin++) {
      report_state(thisPin);
    }
  }
  else if (strPayload == "P01") {

    // Pulserelais aan
    ShowDebug("Enabling pulse relay.");
    digitalWrite(PulseRelayPin, LOW);
    String messageString = "P01";
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out_door, messageBuffer);
    PulseActivityTime = millis();
    ShowDebug(String(PulseActivityTime));
  }
  else if (strPayload[0] == 'L') {
    analogWrite(PWMoutput, strPayload.substring(1).toInt());
  }
  else {

    // Onbekend commando
    ShowDebug("Unknown value");
    mqttClient.publish(topic_out, "Unknown command");
  }
}

void loop() {
  if (!mqttClient.connected()) {
    ShowDebug("Not Connected!");
    reconnect();
  }

  if (((millis() - PulseActivityTime) > PULSE_RELAY_TIME) && digitalRead(PulseRelayPin) == LOW)
  {
    ShowDebug("Disabling pulse relay.");
    ShowDebug(String(PulseActivityTime));
    String messageString = "P00";
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out_door, messageBuffer);
    digitalWrite(PulseRelayPin, HIGH);
  }
  // it's time to send new data?
  if (millis() - previousMillis > PUBLISH_DELAY) {
    previousMillis = millis();
    sendData();
  }
  else {
    for (int id; id < NumberOfButtons; id++) {
      processButtonDigital(id);
    }
  }
  mqttClient.loop();
}
