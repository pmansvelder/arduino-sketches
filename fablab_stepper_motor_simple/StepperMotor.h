#include <Stepper.h>

// for use with 28BYJ-48 stepper motor
// defines a function myStepper.step(<steps>) to turn the stepper a number if steps CCW (positive) or CW (negative)
// also defines a function degToSteps to convert degrees to steps
// stepper motor is connected to pins 8,9,10,11 (In1,In2,In3,In4)

#define motorPin1 8 
#define motorPin2 9
#define motorPin3 10
#define motorPin4 11
#define stepsPerRevolution 2038 // for 28BYJ-48 stepper motor

Stepper myStepper = Stepper(stepsPerRevolution, motorPin1, motorPin3, motorPin2, motorPin4);

float degToSteps(float deg){
  return (stepsPerRevolution) * (deg/360);
}
