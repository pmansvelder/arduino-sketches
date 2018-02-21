#include <NewPing.h>
#include <dht.h>

// Sketch to read sensors based on serial input

dht DHT;

struct tupleint {
  int a;
  int b;
};

int incomingByte = 0;   // for incoming serial data
char incomingSYM, commandChar;
struct tupleint temp_temp;

#define trigPin 52  // HC-SR04 sensor on pin 52/53
#define echoPin 53  //
#define luminancePin 0 // Analog LDR sensor on pin A0
#define dhtPin 51

#define MAX_DISTANCE 200

NewPing sonar(trigPin,echoPin,MAX_DISTANCE);

void setup() {
  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
  Serial.println("OK");
}

int getDistance() { // HC-SR04 sensor
  return sonar.ping_cm();
}

int getDistance_old() { // HC-SR04 sensor
  long duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;
  return distance;
}

int getLuminance() {
  long luminance;
  luminance = analogRead(luminancePin);
  return luminance;
}

struct tupleint getTempHum() {
  struct tupleint temp_hum;
  DHT.read11(dhtPin);
  temp_hum.a = DHT.temperature;
  temp_hum.b = DHT.humidity;
  return temp_hum;
}

void loop() {

  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingSYM = Serial.read();
    if (incomingSYM != '\n') {
      commandChar = incomingSYM;
      switch (commandChar) {
        case 'D':
          Serial.print("Distance: ");
          Serial.println(getDistance());
          break;
        case 'I':
          Serial.print("Luminance: ");
          Serial.println(getLuminance());
          break;
        case 'T':
          Serial.print("Temperature: ");
          temp_temp = getTempHum();
          Serial.println(temp_temp.a);
          break;
        case 'H':
          temp_temp = getTempHum();
          Serial.print("Humidity: ");
          Serial.println(temp_temp.b);
          break;
        default:
          Serial.println("NOP");
          break;
      }
      Serial.println("OK");
    }
  }
}
