#include "MotorSturing.h" / Module voor aansturing motor met L298N H - brug

void setup() {
  // put your setup code here, to run once:
  SetupMotorPins();
  ControlMotor(0, 50);
  ControlMotor(1, -50);
}

void loop() {
}
