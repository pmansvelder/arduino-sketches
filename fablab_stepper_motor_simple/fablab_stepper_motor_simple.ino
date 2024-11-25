#include "StepperMotor.h"

void setup() {  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
	Serial.println("Rotate 90 degrees CCW slowly at 5 RPM");
	myStepper.setSpeed(5);
  myStepper.step(degToSteps(90));
	delay(1000);
	
	Serial.println("Rotate 180 degrees CW quickly at 10 RPM");
	myStepper.setSpeed(10);
  myStepper.step(degToSteps(-180));
	delay(1000);
}
