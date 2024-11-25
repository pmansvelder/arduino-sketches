#include "LDRLineSensor.h"

// this sketch uses the first LDR connected to A0
// GND --[ 10k ]-+--[ LDR ]--- 5V
//               |
//               A0
//
// LED for illumination connected to pin 13

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}

void loop() {
  Serial.print("LDR 0 1: ");
  Serial.print(ShowLDRValue(0));
  Serial.print(" ");
  Serial.print(ShowLDRValue(1));
  if (LightOrDark(0)) {
    Serial.print(" licht");
  } else {
    Serial.print(" donker");
  }
  if (LightOrDark(1)) {
    Serial.println(" licht");
  } else {
    Serial.println(" donker");
  }
  delay(250);
}