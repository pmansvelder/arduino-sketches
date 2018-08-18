const int W5100_RESET_PIN = 8;

#include <Ethernet.h>// Ethernet.h library

#define SERIAL_SPEED 9600
#define MY_IP "192.168.178.233"

void setup() {
  Serial.begin(SERIAL_SPEED);
  Serial.println();
  Serial.println(F("node starting..."));
  pinMode(W5100_RESET_PIN, OUTPUT);
  digitalWrite(W5100_RESET_PIN, LOW);
  delay(100);
  digitalWrite(W5100_RESET_PIN, HIGH);
  // give the Ethernet shield a second to initialize:
  delay(3000);
#ifdef USE_DHCP
  Serial.println(F("DHCP..."));
  while (Ethernet.begin(mac) == 0) {
    Serial.println(F("DHCP failed."));
    delay(10000);
  }
#else
  Serial.print(F("static IP..."));
  Ethernet.begin(mac, IPAddress(MY_IP));
#endif
  Serial.print(F("local IP:"));
  Serial.println(Ethernet.localIP());
  Serial.println(F("node started."));
}
