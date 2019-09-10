#include <DHT.h>
#include <Adafruit_Sensor.h>
#define DHT_PIN 3 // Vul hier de pin in van de DHT11 sensor
DHT dht(DHT_PIN, DHT22);

void setup() {
  // put your setup code here, to run once:
  for (int i = 14; i < 18 ; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH);
  }
  Serial.begin(9600);
  Serial.println("Outputs initialized");
  dht.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float hic = dht.computeHeatIndex(t, h, false);
  Serial.println("Humidity: " + String(h));
  Serial.println("Temperature: " + String(t));
  Serial.println("Heatindex: " + String(hic));
  if (t > 30) {
    for (int i = 14; i < 18 ; i++) {
      Serial.print("Output " + String(i) + " on");
      digitalWrite(i, LOW);
      delay(1000);
      Serial.println(" and off.");
      digitalWrite(i, HIGH);
      delay(1000);
    }
  }
  delay(2000);
}
