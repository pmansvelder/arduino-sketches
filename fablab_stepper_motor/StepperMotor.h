#include <AccelStepper.h>

#define motorPin1 8 
#define motorPin2 9
#define motorPin3 10
#define motorPin4 11

// Settings for 28BYJ-48 type stepper motor
int stepsPerRevolution = 64; // 64 voor 360 graden
float degreePerRevolution = 5.625; // 

AccelStepper stepper(AccelStepper::HALF4WIRE, motorPin1, motorPin3, motorPin2, motorPin4);

float degToSteps(float deg){
  return (stepsPerRevolution / degreePerRevolution) * deg;
}

void SetupStepper() {
  stepper.setMaxSpeed(1000.0);
  stepper.setAcceleration(100.0);
  stepper.setSpeed(200);
}