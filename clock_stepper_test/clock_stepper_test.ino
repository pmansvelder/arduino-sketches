#define MOTOR_PHASE_A 4
#define MOTOR_PHASE_B 5
#define MOTOR_PHASE_C 6
#define MOTOR_PHASE_D 7

// Motor.

#define MOTOR_STEP_FAST                     1           // fast step speed
#define MOTOR_STEP_NORMAL                   4           // normal step speed
#define MOTOR_STEPS_PER_HOUR                4096        // steps for one full revolution


int             nMillisecondsStepHold         = MOTOR_STEP_NORMAL;    // time in milliseconds between motor steps


void setup() {
  // Stepper motor contoller.

  pinMode(MOTOR_PHASE_A, OUTPUT);
  pinMode(MOTOR_PHASE_B, OUTPUT);
  pinMode(MOTOR_PHASE_C, OUTPUT);
  pinMode(MOTOR_PHASE_D, OUTPUT);
}

void  MotorOff()
{
  digitalWrite(MOTOR_PHASE_A, LOW);
  digitalWrite(MOTOR_PHASE_B, LOW);
  digitalWrite(MOTOR_PHASE_C, LOW);
  digitalWrite(MOTOR_PHASE_D, LOW);
}

void  Step(int nDirection)
{
  // Local variables.

  static  int nPhase = 0;

  // Update phase.

  nPhase = ((nDirection < 0) ? (nPhase - 1) : (nPhase + 1)) & 7;

  // Step this phase.

  switch (nPhase)
  {
    case 0:
      {
        digitalWrite(MOTOR_PHASE_D, HIGH);
        digitalWrite(MOTOR_PHASE_C, LOW);
        digitalWrite(MOTOR_PHASE_B, LOW);
        digitalWrite(MOTOR_PHASE_A, LOW);
      }
      break;

    case 1:
      {
        digitalWrite(MOTOR_PHASE_D, HIGH);
        digitalWrite(MOTOR_PHASE_C, HIGH);
        digitalWrite(MOTOR_PHASE_B, LOW);
        digitalWrite(MOTOR_PHASE_A, LOW);
      }
      break;

    case 2:
      {
        digitalWrite(MOTOR_PHASE_D, LOW);
        digitalWrite(MOTOR_PHASE_C, HIGH);
        digitalWrite(MOTOR_PHASE_B, LOW);
        digitalWrite(MOTOR_PHASE_A, LOW);
      }
      break;

    case 3:
      {
        digitalWrite(MOTOR_PHASE_D, LOW);
        digitalWrite(MOTOR_PHASE_C, HIGH);
        digitalWrite(MOTOR_PHASE_B, HIGH);
        digitalWrite(MOTOR_PHASE_A, LOW);
      }
      break;

    case 4:
      {
        digitalWrite(MOTOR_PHASE_D, LOW);
        digitalWrite(MOTOR_PHASE_C, LOW);
        digitalWrite(MOTOR_PHASE_B, HIGH);
        digitalWrite(MOTOR_PHASE_A, LOW);
      }
      break;

    case 5:
      {
        digitalWrite(MOTOR_PHASE_D, LOW);
        digitalWrite(MOTOR_PHASE_C, LOW);
        digitalWrite(MOTOR_PHASE_B, HIGH);
        digitalWrite(MOTOR_PHASE_A, HIGH);
      }
      break;

    case 6:
      {
        digitalWrite(MOTOR_PHASE_D, LOW);
        digitalWrite(MOTOR_PHASE_C, LOW);
        digitalWrite(MOTOR_PHASE_B, LOW);
        digitalWrite(MOTOR_PHASE_A, HIGH);
      }
      break;

    case 7:
      {
        digitalWrite(MOTOR_PHASE_D, HIGH);
        digitalWrite(MOTOR_PHASE_C, LOW);
        digitalWrite(MOTOR_PHASE_B, LOW);
        digitalWrite(MOTOR_PHASE_A, HIGH);
      }
      break;
  }

  // Hold this step for nMillisecondsStepHold milliseconds.

  delay(nMillisecondsStepHold);
}

void loop() {
  // put your main code here, to run repeatedly:
  //  MotorOff();
  for (int i = 0; i < 4096; i++) {
    Step(1);
  }
  MotorOff();
  delay(500);
  for (int i = 0; i < 4096; i++) {
    Step(-1);
  }
  MotorOff();
  delay(500);
}
