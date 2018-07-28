// Sketch to use Adafruit Huzzah ESP8266 as domotica endpoint with MQTT

#include <ESP8266WiFi.h>
#include "PubSubClient.h" //PubSubClient.h Library from Knolleary

const char* ssid     = "LoggerMans";
const char* password = "jukulapikkie";

// Vul hier de naam in waarmee de Arduino zich aanmeldt bij MQTT
#define CLIENT_ID  "domus_esp_meterkast"

String hostname = CLIENT_ID;

// Vul hier het interval in waarmee sensorgegevens worden verstuurd op MQTT
#define PUBLISH_DELAY 5000 // that is 3 seconds interval
#define DEBOUNCE_DELAY 150
#define LONGPRESS_TIME 450

bool debug = false;
bool startsend = HIGH; // flag for sending at startup

// Vul hier de MQTT topic in waar deze arduino naar luistert
const char* topic_in = "domus/esp/in";

// Vul hier de uitgaande MQTT topics in
const char* topic_out = "domus/esp/uit";
const char* topic_out_meter_voltage = "domus/mk/uit/meter/voltage";
const char* topic_out_meter_tariff = "domus/mk/uit/meter/tarief";
const char* topic_out_meter_high = "domus/mk/uit/meter/meterhigh";
const char* topic_out_meter_low = "domus/mk/uit/meter/meterlow";
const char* topic_out_meter_power = "domus/mk/uit/meter/power";
const char* topic_out_meter_gas = "domus/mk/uit/meter/gas";

int NumberOfRelays = 2;
int RelayPins[] = {2, 0};
bool RelayInitialState[] = {LOW, HIGH};

// Vul hier het aantal pulsrelais in
int NumberOfPulseRelays = 1;
// Vul hier de pins in van het pulserelais.
int PulseRelayPins[] = {2};
long PulseActivityTimes[] = {};
// Vul hier de default status in van het pulsrelais (sommige relais vereisen een 0, andere een 1 om te activeren)
bool PulseRelayInitialStates[] = {LOW};
// Vul hier de tijden in voor de pulserelais
const int PulseRelayTimes[] = {250};

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

char messageBuffer[100];
char topicBuffer[100];
String ip = "";

bool pulse_command;

long previousMillis;

WiFiClient espClient;
PubSubClient mqttClient;

void ShowDebug(String tekst) {
  if (debug) {
    Serial.println(tekst);
  }
}

void decodeTelegram() {
  long tl = 0;
  long tld = 0;

  if (Serial.available()) {
    input = Serial.read();
    char inChar = (char)input;
    // Fill buffer up to and including a new line (\n)
    buffer[bufpos] = input & 127;
    bufpos++;

    if (input == '\n') { // We received a new line (data up to \n)
      ShowDebug(buffer);

      // 0-0:96.14 = Elektra tarief (laag = 1)
      if (sscanf(buffer, "0-0:96.14.0(%d" , &tl) == 1) {
        Tariff = tl;
      }

      // 1-0:32.7 = Netspanning
      if (sscanf(buffer, "1-0:32.7.0(%ld.%ld" , &tl, &tld) == 2) {
        tl += tld;
        Voltage = tl;
      }

      // 1-0:1.8.1 = Elektra verbruik laag tarief (DSMR v4.0)
      if (sscanf(buffer, "1-0:1.8.1(%ld.%ld" , &tl, &tld) == 2) {
        tl *= 1000;
        tl += tld;
        mEVLT = tl;
      }

      // 1-0:1.8.2 = Elektra verbruik hoog tarief (DSMR v4.0)
      if (sscanf(buffer, "1-0:1.8.2(%ld.%ld" , &tl, &tld) == 2) {
        tl *= 1000;
        tl += tld;
        mEVHT = tl;
      }

      // 1-0:1.7.0 = Electricity consumption actual usage (DSMR v4.0)
      if (sscanf(buffer, "1-0:1.7.0(%ld.%ld" , &tl , &tld) == 2)
      {
        mEAV = (tl * 1000) + tld;
      }

      // 0-1:24.2.1 = Gas (DSMR v4.0) on Kaifa MA105 meter
      if (strncmp(buffer, "0-1:24.2.1", strlen("0-1:24.2.1")) == 0) {
        if (sscanf(strrchr(buffer, '(') + 1, "%d.%d", &tl, &tld) == 2) {
          mG = (tl * 1000) + tld;
        }
      }

      // Empty buffer again (whole array)
      for (int i = 0; i < 75; i++)
      {
        buffer[i] = 0;
      }
      bufpos = 0;
    }
  } //Einde 'if Serial.available'
} //Einde 'decodeTelegram()' functie

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
  if (debug) {
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
  }
  pinMode(0, OUTPUT);

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
  if (debug) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Netmask: ");
    Serial.println(WiFi.subnetMask());
    Serial.print("Gateway: ");
    Serial.println(WiFi.gatewayIP());
  }
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
    ShowDebug(String(PulseRelayId));
    digitalWrite(PulseRelayPins[PulseRelayId], PulseRelayInitialStates[PulseRelayId]);
    pulse_command = false;
    ShowDebug(String(PulseActivityTimes[PulseRelayId]));
    ShowDebug(String(millis()));
    ShowDebug(String(digitalRead(PulseRelayPins[PulseRelayId])));
    ShowDebug(String(PulseRelayInitialStates[PulseRelayId]));
    String messageString = "P" + String(PulseRelayId) + "0";
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out, messageBuffer);
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

  // Send status of relays
  for (int thisPin = 0; thisPin < NumberOfRelays; thisPin++) {
    report_state(thisPin);
  }
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
      digitalWrite(0, LOW);
      // Once connected, publish an announcement...
      mqttClient.publish(topic_out, ip.c_str());
      mqttClient.publish(topic_out, "ESP8266 connected");
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
  }

  else if (strPayload == "IP")  {
    // 'Show IP' commando
    mqttClient.publish(topic_out, ip.c_str()); // publish IP nr
    mqttClient.publish(topic_out, hostname.c_str()); // publish hostname
  }
  else if (strPayload[0] == 'P') {
    pulse_command = true;
    int PulseRelayPort = strPayload[1] - 48;

    // Pulserelais aan
    ShowDebug("Enabling pulse relay " + String(PulseRelayPort) + ".");
    digitalWrite(PulseRelayPins[PulseRelayPort], !PulseRelayInitialStates[PulseRelayPort]);
    String messageString = "P" + String(PulseRelayPort) + "1";
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out, messageBuffer);
    PulseActivityTimes[PulseRelayPort] = millis();
    ShowDebug("Timestamp at beginning of pulse");
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

  // ...see if it's time to send new data, ....
  if (millis() - previousMillis > PUBLISH_DELAY) {
    previousMillis = millis();
    sendData();
  }

  // ...handle the PulseRelays, ...
  if (pulse_command) {
    for (int id; id < NumberOfPulseRelays; id++) {
      ProcessPulseRelays(id);
    }
  }
  decodeTelegram();
  mqttClient.loop();
}
