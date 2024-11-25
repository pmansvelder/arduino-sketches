// initial pin layout for 2 motors

int MotorPin1[2] = { 2, 4 };
int MotorPin2[2] = { 3, 5 };
int MotorPwm[2] = { 9, 10 };

// Set up motor pins

void SetupMotorPins() {
  for (int i = 0; i < 2; i++) {
    pinMode(MotorPin1[i], OUTPUT);
    pinMode(MotorPin2[i], OUTPUT);
    pinMode(MotorPwm[i], OUTPUT);
  }
}

// function to control motor connected to L298N
// parameters:
// Motor: 0 for motor 1, 1 for motor 2
// Speed: 0-100 for motor speed

void ControlMotor(int Motor, int Speed) {
  int IntSpeed = map(abs(Speed), 0, 100, 100, 255);
  if (Speed == 0) {
    IntSpeed = 0;
  }
  analogWrite(MotorPwm[Motor], IntSpeed);
  if (Speed > 0) {  // motor goes FORWARD
    digitalWrite(MotorPin1[Motor], HIGH);
    digitalWrite(MotorPin2[Motor], LOW);
  } else {  // motor goes BACKWARD
    digitalWrite(MotorPin1[Motor], LOW);
    digitalWrite(MotorPin2[Motor], HIGH);
  }
}

void StopMotor(int Motor) {
  analogWrite(MotorPwm[Motor], 0);
}