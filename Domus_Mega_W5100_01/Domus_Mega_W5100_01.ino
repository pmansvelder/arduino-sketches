/*
 *        <========Arduino Sketch for Arduino Mega with Ethernet shield W5100=========>
 *        
          Arduino UNO with W5100 Ethernetshield or W5100 Ethernet module, used as MQTT client
          It will connect over Wifi to the MQTT broker and controls digital outputs (LED, relays)
          The topics have the format "domus_mqtt/up" for outgoing messages and  "domus_mqtt/down" for incoming messages
          As the available memory of a UNO  with Ethernetcard is limited, I have kept the topics short
          Also, the payloads  are kept short
          The outgoing topics are
          /domus_mqtt/up
          Here, eacht relay state is reported using the same syntax as the switch command:
          R<relay number><state>

          There is only one incoming topic:
          /domus_mqtt/down
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
          - add extra button on input A5(19)
          - use output 17 as a PWM channel for the button LEDs
          - implement short/long press for buttons

*/
#include <Ethernet.h>// Ethernet.h library
#include "PubSubClient.h" //PubSubClient.h Library from Knolleary
#define CLIENT_ID       "SW1_huiskamer"
#define PUBLISH_DELAY   3000 // that is 3 seconds interval
#define DEBOUNCE_DELAY 200

String hostname = "domus_huiskamer";

int NumberOfRelays = 10;
int RelayPins[] = {9, 8, 7, 6, 5, 3, 14, 15, 16, 17};

int NumberOfButtons = 2;
int ButtonPins[] = {18, 19};
static byte lastButtonStates[] = {0, 0};
long lastActivityTimes[] = {0, 0};

char messageBuffer[100];
char topicBuffer[100];
//char clientBuffer[50];

String ip = "";
bool statusKD = HIGH;
bool statusBD = HIGH;
bool statusGD = HIGH;
bool relaystate1 = LOW;
bool pir = LOW;
bool startsend = HIGH;// flag for sending at startup
bool debug = true;
int lichtstatus; //contains LDR reading
uint8_t mac[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x07};

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
  }

  for (int thisButton = 0; thisButton < NumberOfButtons; thisButton++) {
    pinMode(ButtonPins[thisButton], INPUT_PULLUP);
  }

  // setup serial communication

  if (debug) {
    Serial.begin(9600);
    while (!Serial) {};

    ShowDebug(F("MQTT Arduino Demo"));
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
  mqttClient.setServer( "192.168.178.37", 1883); // or local broker
  ShowDebug(F("MQTT client configured"));
  mqttClient.setCallback(callback);
  ShowDebug("");
  ShowDebug(F("Ready to send data"));
  previousMillis = millis();
  mqttClient.publish("/domus_mqtt/up", ip.c_str());
}

void loop() {

  // it's time to send new data?
  if (millis() - previousMillis > PUBLISH_DELAY) {
    sendData();
    previousMillis = millis();
  }
  else {
 //   processButtonDigital(ButtonPin);
    for (int id; id < NumberOfButtons; id++) {
      processButtonDigital(id);
    }
  }
  mqttClient.loop();
}

void processButtonDigital( int buttonId )
{
  int sensorReading = digitalRead( ButtonPins[buttonId] );

  if ( sensorReading == 0 ) // Input pulled low to GND. Button pressed.
  {

    if ( lastButtonStates[buttonId] == 0 )  // The button was previously un-pressed
    {
      if ((millis() - lastActivityTimes[buttonId]) > DEBOUNCE_DELAY) // Proceed if we haven't seen a recent event on this button
      {
        lastActivityTimes[buttonId] = millis();
        Serial.println( "Button" + String(buttonId) + " pressed" );
        String messageString = "Button" + String(buttonId);
        messageString.toCharArray(messageBuffer, messageString.length() + 1);
        mqttClient.publish("/domus_mqtt/up", messageBuffer);
      }
    }
    lastButtonStates[buttonId] = 1;
  }
  else {
    lastButtonStates[buttonId] = 0;
  }
}

void sendData() {
  char msgBuffer[20];
  ShowDebug("%");
  ShowDebug("Relay is: ");
  ShowDebug((relaystate1 == LOW) ? "OPEN" : "CLOSED");
  if (mqttClient.connect(CLIENT_ID)) {
    mqttClient.subscribe("/domus_mqtt/down");
    if (startsend) {
      mqttClient.publish("/domus_mqtt/up", ip.c_str());
      startsend = LOW;
    }
  }
}

void report_state(int outputport)
{
  String messageString = "R" + String(outputport) + String(digitalRead(RelayPins[outputport]));
  messageString.toCharArray(messageBuffer, messageString.length() + 1);
  mqttClient.publish("/domus_mqtt/up", messageBuffer);
}

void callback(char* topic, byte* payload, unsigned int length) {
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
    ShowDebug("Extended command");

    RelayPort = strPayload[1] - 48;
    if (RelayPort > 16) RelayPort -= 3;
    RelayValue = strPayload[2] - 48;

    ShowDebug(String(RelayPort));
    ShowDebug(String(RelayValue));
    ShowDebug(String(HIGH));

    if (RelayValue == 40) {
      ShowDebug("Toggling...");
      digitalWrite(RelayPins[RelayPort], !digitalRead(RelayPins[RelayPort]));
    } else {
      digitalWrite(RelayPins[RelayPort], RelayValue);
    }
    report_state(RelayPort);
  } else if (payload[0] == 50)
  {
    mqttClient.publish("/domus_mqtt/up", ip.c_str());// publish IP nr
  }
  else if (strPayload == "AON")
  {
    for (int i = 0 ; i < NumberOfRelays; i++) {
      digitalWrite(RelayPins[i], 1);
      report_state(i);
    }
  }
  else if (strPayload == "AOF")
  {
    for (int i = 0 ; i < NumberOfRelays; i++) {
      digitalWrite(RelayPins[i], 0);
      report_state(i);
    }
  }
  else if (strPayload == "STAT")
  {
    for (int thisPin = 0; thisPin < NumberOfRelays; thisPin++) {
      report_state(thisPin);
    }
  }
  else {
    ShowDebug("Unknown value");
    mqttClient.publish("/domus_mqtt/up", "Unknown command");
  }

}
