/*
  <========Arduino Sketch for Adafruit Huzzah ESP8266=========>
*/
#include <ESP8266WiFi.h>
#include "secrets.h"
#include "PubSubClient.h"

const char* ssid     = SECRET_SSID;
const char* password = SECRET_PASS;

// Vul hier de naam in waarmee de Arduino zich aanmeldt bij MQTT
#define CLIENT_ID  "domus_esp_huiskamer"

String hostname = CLIENT_ID;

bool debug = true;
bool startsend = HIGH; // flag for sending at startup

// Vul hier de MQTT topic in waar deze arduino naar luistert
const char* topic_in = "domus/esp/in";

// Vul hier de uitgaande MQTT topics in
const char* topic_out = "domus/esp/uit";
const char* topic_out_brightness = "domus/esp/uit/brightness";

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

// For ESP8266
#define LED_BLUE 2
#define LED_PWM_PIN 16

#define EN_PIN_A 5
#define EN_PIN_B 4
#define BTN_PIN 0

unsigned char encoder_A;
unsigned char encoder_B;
unsigned char last_encoder_A;
unsigned char last_encoder_B;
int led_power = 0;
int power_step = 1;
long loop_time;
long button_time;
bool last_button_state;

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    digitalWrite(LED_BLUE, HIGH);
    ShowDebug("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(CLIENT_ID)) {
      ShowDebug("connected");
      digitalWrite(LED_BLUE, LOW);
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

void report_state()
{
  String messageString;
  ShowDebug("LED Power: " + String(led_power));
  if (led_power == 0) {
    messageString = "OFF";
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out, messageBuffer);
  }
  else {
    messageString = "ON";
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out, messageBuffer);
    messageString = String(led_power);
    messageString.toCharArray(messageBuffer, messageString.length() + 1);
    mqttClient.publish(topic_out_brightness, messageBuffer);
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

  if (strPayload[0] == 'OFF') {
    led_power = 0;
    analogWrite(LED_PWM_PIN, led_power);
    report_state();
  }
  else if (strPayload[0] == 'ON') {
    led_power = 255;
    analogWrite(LED_PWM_PIN, led_power);
    report_state();
  }
  else if (strPayload == "IP")  {
    // 'Show IP' commando
    mqttClient.publish(topic_out, ip.c_str()); // publish IP nr
    mqttClient.publish(topic_out, hostname.c_str()); // publish hostname
  }
  else {
    led_power = strPayload.substring(0).toInt();
    analogWrite(LED_PWM_PIN, led_power);
    report_state();
  }
}

void setup() {

  Serial.begin( 115200 );
  delay(100);
  Serial.println( "Domus LED PWM dimmer START" );

  ShowDebug("Connecting to ");
  ShowDebug(ssid);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(EN_PIN_A, INPUT_PULLUP);
  pinMode(EN_PIN_B, INPUT_PULLUP);
  pinMode(BTN_PIN, INPUT_PULLUP);

  analogWrite(LED_PWM_PIN, 0);

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
  digitalWrite(LED_BLUE, LOW);
}

void loop() {
  if (!mqttClient.connected()) {
    ShowDebug("Not Connected!");
    reconnect();
  }
  long current_time = millis(); //millis() - Returns the number of milliseconds since the Arduino board began running the current program.

  bool btn = digitalRead(BTN_PIN);  //Read state of encoder switch.
  if ( btn != last_button_state && current_time - button_time > 100 ) { //If the state is different from the previous one and since last state change has been at least 100ms.
    if ( btn == LOW && last_button_state ) { //Detect the transitions from HIGH to LOW
      if ( led_power == 255 ) {
        led_power = 0;
        Serial.println( "L: " + String(led_power) );
      } else {
        led_power = 255;
        Serial.println( "L: " + String(led_power) );
      }
      analogWrite(LED_PWM_PIN, led_power);
      report_state();
    }
    last_button_state = btn;
    button_time = current_time;
  }

  if ( current_time - loop_time >= 5 ) {
    encoder_A = digitalRead(EN_PIN_A);  //Read encoder pin A
    encoder_B = digitalRead(EN_PIN_B);  //Read encoder pin B

    if ( !encoder_A && last_encoder_A ) {
      if ( encoder_B ) {
        Serial.println( "L: " + String(led_power) );
        led_power = led_power - power_step;
      } else {
        Serial.println( "R: " + String(led_power) );
        led_power = led_power + power_step;
      }

      if ( led_power < 0 ) led_power = 0;
      if ( led_power >= 255 ) led_power = 255;

      if ( led_power >= 0 && led_power <= 10 ) {
        power_step = 1;
      } else if ( led_power > 10 && led_power <= 20 ) {
        power_step = 2;
      } else if ( led_power > 20 && led_power <= 30 ) {
        power_step = 5;
      } else if ( led_power > 30 ) {
        power_step = 10;
      }
      analogWrite(LED_PWM_PIN, led_power);
      report_state();
    }

    last_encoder_A = encoder_A;
    last_encoder_B = encoder_B;

    loop_time = current_time;
  }
  mqttClient.loop();
}
