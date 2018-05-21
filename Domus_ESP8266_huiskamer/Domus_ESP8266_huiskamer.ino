// Sketch to use Adafruit Huzzah ESP8266 as domotica endpoint with MQTT

#include <ESP8266WiFi.h>
#include "PubSubClient.h" //PubSubClient.h Library from Knolleary

const char* ssid     = "LoggerMans";
const char* password = "jukulapikkie";

// Vul hier de naam in waarmee de Arduino zich aanmeldt bij MQTT
#define CLIENT_ID  "domus_esp_huiskamer"

String hostname = CLIENT_ID;

bool debug = true;
bool startsend = HIGH; // flag for sending at startup

// Vul hier de MQTT topic in waar deze arduino naar luistert
const char* topic_in = "domus/esp/in";

// Vul hier de uitgaande MQTT topics in
const char* topic_out = "domus/esp/uit";

int NumberOfRelays = 1;
int RelayPins[] = {0};
bool RelayInitialState[] = {HIGH};

// Vul hier het aantal pulsrelais in
int NumberOfPulseRelays = 0;
// Vul hier de pins in van het pulserelais.
int PulseRelayPins[] = {0};
long PulseActivityTimes[] = {};
// Vul hier de default status in van het pulsrelais (sommige relais vereisen een 0, andere een 1 om te activeren)
bool PulseRelayInitialStates[] = {HIGH};
// Vul hier de tijden in voor de pulserelais
const int PulseRelayTimes[] = {250};

char messageBuffer[100];
char topicBuffer[100];
String ip = "";

WiFiClient espClient;
PubSubClient mqttClient;

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

  for (int thisPin = 0; thisPin < NumberOfPulseRelays; thisPin++) {
    pinMode(PulseRelayPins[thisPin], OUTPUT);
    digitalWrite(PulseRelayPins[thisPin], PulseRelayInitialStates[thisPin]);
  }

  Serial.begin(115200);
  delay(100);

  // We start by connecting to a WiFi network

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  pinMode(0, OUTPUT);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  //convert ip Array into String
  ip = String (WiFi.localIP()[0]);
  ip = ip + ".";
  ip = ip + String (WiFi.localIP()[1]);
  ip = ip + ".";
  ip = ip + String (WiFi.localIP()[2]);
  ip = ip + ".";
  ip = ip + String (WiFi.localIP()[3]);

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Netmask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("Gateway: ");
  Serial.println(WiFi.gatewayIP());

  // setup mqtt client
  mqttClient.setClient(espClient);
  mqttClient.setServer( "majordomo", 1883); // or local broker
  ShowDebug(F("MQTT client configured"));
  mqttClient.setCallback(callback);
}

void ProcessPulseRelays(int PulseRelayId) {
  // Process the timers of the pulse relays and see if we have to close them.
  if (((millis() - PulseActivityTimes[PulseRelayId]) > PulseRelayTimes[PulseRelayId]) && digitalRead(PulseRelayPins[PulseRelayId]) == !PulseRelayInitialStates[PulseRelayId])
  {
    ShowDebug("Disabling pulse relay" + String(PulseRelayId) + ".");
    ShowDebug(String(PulseActivityTimes[PulseRelayId]));
    String messageString = "P" + String(PulseRelayId) + "0";
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out, messageBuffer);
    digitalWrite(PulseRelayPins[PulseRelayId], PulseRelayInitialStates[PulseRelayId]);
  }
}

void report_state(int outputport)
{
  String messageString = "R" + String(outputport) + String(digitalRead(RelayPins[outputport]));
  messageString.toCharArray(messageBuffer, messageString.length() + 1);
  mqttClient.publish(topic_out, messageBuffer);
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

  if (strPayload[0] == 'R') {
    digitalWrite(0, !digitalRead(0));
    report_state(0);
  }
  else if (strPayload == "IP")  {
    // 'Show IP' commando
    mqttClient.publish(topic_out, ip.c_str()); // publish IP nr
  }
  else if (strPayload[0] == 'P') {

    int PulseRelayPort = strPayload[1] - 48;

    // Pulserelais aan
    ShowDebug("Enabling pulse relay " + String(PulseRelayPort) + ".");
    digitalWrite(PulseRelayPins[PulseRelayPort], !PulseRelayInitialStates[PulseRelayPort]);
    String messageString = "P" + String(PulseRelayPort) + "1";
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out, messageBuffer);
    PulseActivityTimes[PulseRelayPort] = millis();
    ShowDebug(String(PulseActivityTimes[PulseRelayPort]));
  }

}

void loop() {
  // put your main code here, to run repeatedly:
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
//  for (int id; id < NumberOfPulseRelays; id++) {
//    ProcessPulseRelays(id);
//  }

  mqttClient.loop();
}
