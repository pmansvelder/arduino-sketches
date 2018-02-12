/*
          Arduino UNO with W5100 Ethernetshield or W5100 Ethernet module, used as MQTT client
          It will connect over Wifi to the MQTT broker and controls digital outputs (LED, relays)
          and gives the Temperature and Humidity, as well as the state of some switches
          The topics have the format "home/br/sb" for southbound messages and  "home/nb" for northbound messages
          Southbound are messages going to the client, northbound are messages coming from the client
          As the available memory of a UNO  with Ethernetcard is limited, I have kept the topics short
          Also, the payloads  are kept short
          The Northbound topics are
          home/br/nb/temp  for temperature
          home/br/nb/humid  for humidity
          home/br/nb/deur  for a door switch
          home/br/nb/l for  the lightintensity
          home/br/nb/pr  for the status of a PIR sensor
          home/br/nb/ip showing the IP number of the client
          home/br/nb/relay showing the relaystate

          There is only one southbound topic:
          home/br/sb
          The payload here determines the action:
          0 -Switch the relay off
          1-Switch the  relay on
          2-Publish the IP number of the client
          3 Ask for the relaystate// REMOVED

          On Startup, the Client publishes the IP number

          Adapted 4-2-2018 by Peter Mansvelder:

          removed Temp/Humidity, added multiple relays for MQTT house control
          To Do: use lookup table for output pins, using those not used by ethernet shield (4, 10, 11, 12, 13) and other 
          ports (0, 1 used by serial interface). I used the following ports:
          3,5,6,7,8,9,A0(14),A1(15),A2(16),A3(17)
          
*/
#include <Ethernet.h>// Ethernet.h library
#include "PubSubClient.h" //PubSubClient.h Library from Knolleary
#define CLIENT_ID       "SW1_huiskamer"
#define PUBLISH_DELAY   5000 // that is 3 seconds interval
#define ledPin 13

String hostname = "domus_huiskamer";
int RelayPins[] = {9, 8, 7, 6, 5, 3, 14, 15, 16, 17};
int NumberOfRelays = 10;
String ip = "";
bool statusKD = HIGH;
bool statusBD = HIGH;
bool statusGD = HIGH;
bool relaystate1 = LOW;
bool pir = LOW;
bool startsend = HIGH;// flag for sending at startup
bool debug = true;
int lichtstatus; //contains LDR reading
uint8_t mac[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x06};

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

  // setup serial communication

  Serial.begin(9600);
  while (!Serial) {};

  ShowDebug(F("MQTT Arduino Demo"));
  ShowDebug("");
  ShowDebug(String(A0));

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
  // mqttClient.setServer("test.mosquitto.org", 1883);//use public broker
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
  mqttClient.loop();
}

void sendData() {
  char msgBuffer[20];
  ShowDebug("%");
  ShowDebug("Relay is: ");
  ShowDebug((relaystate1 == LOW) ? "OPEN" : "CLOSED");
  if (mqttClient.connect(CLIENT_ID)) {
    for (int thisPin = 0; thisPin < NumberOfRelays; thisPin++) {
      mqttClient.publish("/domus_mqtt/up", String(thisPin).c_str());
      mqttClient.publish("/domus_mqtt/up", (digitalRead(RelayPins[thisPin]) == LOW) ? "OPEN" : "CLOSED");
    }
    mqttClient.subscribe("/domus_mqtt/down");
    if (startsend) {
      mqttClient.publish("/domus_mqtt/up", ip.c_str());
      startsend = LOW;
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  char msgBuffer[20];
  // I am only using one ascii character as command, so do not need to take an entire word as payload
  // However, if you want to send full word commands, uncomment the next line and use for string comparison
  payload[length]='\0';// terminate string with 0
  String strPayload = String((char*)payload);  // convert to string
  ShowDebug(strPayload); 
  ShowDebug("Message arrived");
  ShowDebug(topic);
  ShowDebug("] ");//MQTT_BROKER
  for (int i = 0; i < length; i++) {
    ShowDebug(String((char)payload[i]));
  }
  ShowDebug("");
  ShowDebug(String(payload[0]));

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

    digitalWrite(RelayPins[RelayPort], RelayValue);
  } else if (payload[0] == 50)
  {
    mqttClient.publish("/domus_mqtt/up", ip.c_str());// publish IP nr
  }
  else if (strPayload == "AON")
  {
    for (int i = 0 ; i < 10; i++) { 
      digitalWrite(RelayPins[i], 1);
    }
  }
  else if (strPayload == "AOF")
  {
    for (int i = 0 ; i < 10; i++) { 
      digitalWrite(RelayPins[i], 0);
    }
  }
  else{
    ShowDebug("Unknown value");
    mqttClient.publish("/domus_mqtt/up", "Syntax Error");
  }

}

