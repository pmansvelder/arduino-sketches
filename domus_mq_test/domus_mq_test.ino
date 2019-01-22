//#define BMP280 0 // use BMP280 sensor
//#define DHT_present 1 // use DHT sensor
#define MQ_present 1 // MQ-x gas sensor
#define DEBUG 1 // Zet debug mode aan

#if defined(DHT_present)
#include <DHT.h>
#define DHT_PIN 3 // Vul hier de pin in van de DHT11 sensor
DHT dht(DHT_PIN, DHT22);
#endif

#if defined(MQ_present)
#define MQ_PIN A3 // Vul hier de pin in van de MQ sensor
#endif

#if defined(BMP280)
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; // I2C
bool bmp_present = true;
#endif

#if defined(DEBUG)
bool debug = true;
#else
bool debug = false;
#endif

void ShowDebug(String tekst) {
  if (debug) {
    Serial.println(tekst);
  }
}

void setup() {
  // put your setup code here, to run once:
  if (debug) {
    Serial.begin(9600);
    ShowDebug(F("Arduino Domus Test"));
  }
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
}

void loop() {
  // put your main code here, to run repeatedly:
#if defined(MQ_present)
  analogWrite(MQ_PIN, HIGH);
  delay(50);
  byte g = analogRead(MQ_PIN);
  ShowDebug("CO value = " + String(g));
#endif
}
