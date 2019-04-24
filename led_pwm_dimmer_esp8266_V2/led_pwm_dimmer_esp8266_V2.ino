/*
  <========Arduino Sketch for Adafruit Huzzah ESP8266=========>
  Used as a 'smart' LED PWM dimmer for 12V leds
  Interfaces: encoder with button function, connected to pins 0 (button, 5 and 4 (encoder) and WiFi connection to MQTT
  Output 16 is the PWM signal for the Power MOSFET (IRF540)
  Output 2 is the blue LED on the huzzah board, to signal connection to the Wifi network
  Input 0 is the button, connected to the encoder button; it is also connected to the red led on the huzzah board.
  The button has multiple functions:
  - Short press (< 450ms) toggles light between last brightness and off
  - Long press either saves last brightness (when the led was on) and turns it off or sets brightness to 100% (when the led was off)
  The dimmer uses MQTT to listen for commands:
  topic_in for incoming commands (OFF, ON or brightness)
  topic_out for state (ON, OFF) and startup state
  topic_out_brightness for brightness value (0..MAXRANGE)

  v2 for sonoff print from broken S20:
  - GPIO 12 is blue led
  - GPIO 0 is red led

*/
#include <ESP8266WiFi.h>
#include "secrets.h"
#include "PubSubClient.h"

#define MAXRANGE 1023 // max pwm value
#define STEP1 40
#define STEP2 80
#define STEP3 120

const char* ssid     = SECRET_SSID;
const char* password = SECRET_PASS;

// Client ID for MQTT
#define CLIENT_ID  "domus_dimmer_s20"

String hostname = CLIENT_ID;

bool debug = true;

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
#define LED_BLUE 12      // pin for blue led: HIGH is on
#define LED_PWM_PIN 13   // pin for green led: LOW is on

#define EN_PIN_A 5
#define EN_PIN_B 4
#define BTN_PIN 0         // pin for button

unsigned char encoder_A;
unsigned char encoder_B;
unsigned char last_encoder_A;
unsigned char last_encoder_B;
int led_power = 0;
int last_led_power = MAXRANGE;
int power_step = 1;
long loop_time;

// Button parameters
int NumberOfButtons = 1;
int ButtonPins[] = {BTN_PIN};
static byte lastButtonStates[] = {0};
long lastActivityTimes[] = {0};
long LongPressActive[] = {0};
#define DEBOUNCE_DELAY 100
#define LONGPRESS_TIME 450

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    digitalWrite(LED_BLUE, LOW);  // Blue led off: no connection yet..
    ShowDebug("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(CLIENT_ID)) {
      ShowDebug("connected");
      digitalWrite(LED_BLUE, HIGH); // Blue led on: connected to MQTT
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

  if (strPayload == "OFF") {
    last_led_power = led_power;
    led_power = 0;
    analogWrite(LED_PWM_PIN, led_power);
    report_state();
  }
  else if (strPayload == "ON") {
    led_power = last_led_power;
    analogWrite(LED_PWM_PIN, led_power);
    report_state();
  }
  else if (strPayload == "STAT")  {
    report_state();
  }
  else if (strPayload == "IP")  {
    // 'Show IP' commando
    mqttClient.publish(topic_out, ip.c_str()); // publish IP nr
    mqttClient.publish(topic_out, hostname.c_str()); // publish hostname
  }
  else {
    if ((strPayload.substring(0).toInt() == 0) and (strPayload != "0")) {
      mqttClient.publish(topic_out, "Unknown Command"); // unknown command
    }
    else {
      led_power = strPayload.substring(0).toInt();
      if (led_power > MAXRANGE) {
        led_power = MAXRANGE;
      }
      analogWrite(LED_PWM_PIN, led_power);
      report_state();
    }
  }
}

void processButtonDigital( int buttonId )  // routine to check for short or long press, and act accordingly
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
    else if ((millis() - lastActivityTimes[buttonId] > LONGPRESS_TIME) && (!LongPressActive[buttonId])) // Button long press action
    {
      LongPressActive[buttonId] = true;
      ShowDebug( "Button" + String(buttonId) + " long pressed" );
      if ( led_power == 0 ) {                            // If light is off, set brightness to max
        led_power = MAXRANGE;
        Serial.println( "L: " + String(led_power) );
      } else {                                           // if light is on, save brightness and turn light off
        last_led_power = led_power;
        led_power = 0;
        Serial.println( "L: " + String(led_power) );
      }
      analogWrite(LED_PWM_PIN, led_power);
      report_state();
    }
    lastButtonStates[buttonId] = HIGH;
  }
  else {
    if (lastButtonStates[buttonId] == HIGH) {
      if (LongPressActive[buttonId]) {
        LongPressActive[buttonId] = false;
      } else {
        if ((millis() - lastActivityTimes[buttonId]) > DEBOUNCE_DELAY) // Button short press action
        {
          ShowDebug( "Button" + String(buttonId) + " pressed" );
          if ( led_power == 0 ) {                         // If light is off, set brightness to last saved value
            led_power = last_led_power;
            Serial.println( "L: " + String(led_power) );
          } else {                                        // if light is on, turn it off (and don't save brightness)
            led_power = 0;
            Serial.println( "L: " + String(led_power) );
          }
          analogWrite(LED_PWM_PIN, led_power);            // make it so
          report_state();                                 // and report it back to MQTT
        }
      }
      lastButtonStates[buttonId] = LOW;
    }
  }
}

void setup() {

  // set up PWM range

  analogWriteRange(MAXRANGE);

  // set up serial comms

  Serial.begin( 115200 );

  // wait a bit

  delay(100);

  // and send some output

  ShowDebug("Domus LED PWM dimmer START");
  ShowDebug("Connecting to ");
  ShowDebug(ssid);

  // Set gpio pins

  pinMode(LED_BLUE, OUTPUT);
  pinMode(EN_PIN_A, INPUT_PULLUP);
  pinMode(EN_PIN_B, INPUT_PULLUP);
  pinMode(BTN_PIN, INPUT_PULLUP);

  // Set brightness to 0

  analogWrite(LED_PWM_PIN, 0);

  // Connect to Wifi

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

//  ShowDebug("WiFi connected");
//  ShowDebug("IP address: " + String(WiFi.localIP()));
//  ShowDebug("Netmask: " + String(WiFi.subnetMask()));
//  ShowDebug("Gateway: " + String(WiFi.gatewayIP()));

  // setup mqtt client
  
  mqttClient.setClient(espClient);
  mqttClient.setServer( MQTTSERVER, MQTTPORT ); // or local broker
  ShowDebug(F("MQTT client configured"));
  mqttClient.setCallback(callback);

  // and enable blue led to show connected status
  
  digitalWrite(LED_BLUE, HIGH);
}

void loop() {
  if (!mqttClient.connected()) {          // check MQTT connection status and reconnect if connection lost
    ShowDebug("Not Connected!");
    reconnect();
  }
  long current_time = millis();           // millis() - Returns the number of milliseconds since the Arduino board began running the current program.
  processButtonDigital(0);                // check if the pushbutton is pressed
  if ( current_time - loop_time >= 5 ) {  // check every 5 ms for encoder rotation
    encoder_A = digitalRead(EN_PIN_A);    // Read encoder pin A
    encoder_B = digitalRead(EN_PIN_B);    // Read encoder pin B
    if ( !encoder_A && last_encoder_A ) { // if encoder has moved CCW, lower led brightness
      if ( encoder_B ) {
        Serial.println( "L: " + String(led_power) );
        led_power = led_power - power_step;
      } else {                            // if encoder has moved CW, inrease led brightness
        Serial.println( "R: " + String(led_power) );
        led_power = led_power + power_step;
      }
      if ( led_power < 0 ) led_power = 0; // keep led power within range
      if ( led_power >= MAXRANGE ) led_power = MAXRANGE;
      if ( led_power >= 0 && led_power <= STEP1 ) {             // use different steps for different values
        power_step = 1;
      } else if ( led_power > STEP1 && led_power <= STEP2 ) {
        power_step = 2;
      } else if ( led_power > STEP2 && led_power <= STEP3 ) {
        power_step = 5;
      } else if ( led_power > STEP3 ) {
        power_step = 10;
      }
      analogWrite(LED_PWM_PIN, led_power);  // and write to the PWM output
      report_state();                       // report back on the MQTT topic
    }
    last_encoder_A = encoder_A;             // save encoder positions
    last_encoder_B = encoder_B;
    loop_time = current_time;
  }
  mqttClient.loop();
}
