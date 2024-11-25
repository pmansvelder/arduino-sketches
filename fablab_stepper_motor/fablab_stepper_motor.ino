#include "StepperMotor.h"

void setup() {  // put your setup code here, to run once:
  Serial.begin(9600);
  SetupStepper();
}

void loop() {
  for (int i = 0; i < 360; i++) {
    stepper.moveTo(degToSteps(i));  // initial position of stepper motor
    stepper.run();
    delay(100);
  }
}
