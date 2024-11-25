#include "LDRLineSensor.h"

// this sketch uses the first LDR connected to A0
// GND --[ 10k ]-+--[ LDR ]--- 5V
//               |
//               A0
//
#include "MotorSturing.h" / Module voor aansturing motor met L298N H - brug

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SetupMotorPins();
  ControlMotor(0, -50);
  ControlMotor(1, -50);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}

void loop() {
  Serial.print("LDR 0 : ");
  Serial.print(ShowLDRValue(0));
  if (LightOrDark(0)) {
    Serial.println("licht");
    ControlMotor(0, 0);
    ControlMotor(1, 0);
  } else {
    Serial.println("donker");
    ControlMotor(0, -50);
    ControlMotor(1, -50);
  }
  Serial.print("LDR 1 : ");
  Serial.print(ShowLDRValue(1));
  if (LightOrDark(1)) {
    Serial.println("licht");
    ControlMotor(0, 0);
    ControlMotor(1, 0);
  } else {
    Serial.println("donker");
    ControlMotor(0, -50);
    ControlMotor(1, -50);
  }
  delay(250);
}
