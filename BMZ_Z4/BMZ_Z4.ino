// program to move the navigation display 
// on a BMW Z4 using 4 buttons

#define OPEN_BUTTON 6
#define CLOSE_BUTTON 7
#define STEP_OPEN 2
#define STEP_CLOSE 4
#define MOTOR_DIR1 12
#define MOTOR_DIR2 13
#define MOTOR_BRAKE1 9
#define MOTOR_BRAKE2 8
#define CHANNEL_A 3
#define CHANNEL_B 11
#define PWM_SPEED 255
#define SETUP_PIN1 5
// SETUP_PIN1 must be connected to a switch and ground, to set the zero position
// switch on + startup/reset is used to move display freely
#define SETUP_PIN2 10
// SETUP_PIN2 must be connected to a optocoupler and ground, to a sense voltage that is high when contact is made
#define MAX_STEPS 50
// This is the number of steps from 'down' to 'up' position

int min_open = 0;
int max_open = MAX_STEPS;
int current_pos = 0;
boolean limits = true;
boolean action = true;

void setup()
{
  //initialize position
  pinMode(SETUP_PIN1, INPUT_PULLUP);
  if (digitalRead(SETUP_PIN1)==LOW)
  {
    limits = false;
  }
  current_pos = min_open;
  pinMode(SETUP_PIN2, INPUT_PULLUP);
  //establish switch pins
  pinMode(OPEN_BUTTON, INPUT_PULLUP);
  pinMode(CLOSE_BUTTON, INPUT_PULLUP);
  pinMode(STEP_OPEN, INPUT_PULLUP);
  pinMode(STEP_CLOSE, INPUT_PULLUP);
  //establish motor direction toggle pins
  pinMode(MOTOR_DIR1, OUTPUT);
  pinMode(MOTOR_DIR2, OUTPUT);
  //establish motor brake pins
  pinMode(MOTOR_BRAKE1, OUTPUT);
  pinMode(MOTOR_BRAKE2, OUTPUT);
}

void loop(void)
{
  int switchValue;
  switchValue = digitalRead(SETUP_PIN2);
  // SETUP_PIN2 must be connected to a optocoupler
  if (switchValue==LOW && action)
  {
    screenUp();
    action = false;
  }
  if (switchValue==HIGH && !action)
  { 
    screenDown();
    action = true;
  }
  switchValue = digitalRead(OPEN_BUTTON);
  if (switchValue==LOW)
  {
    screenUp();
  }
  switchValue = digitalRead(CLOSE_BUTTON);
  if (switchValue==LOW)
  {
    screenDown();
  }
  switchValue = digitalRead(STEP_OPEN);
  if (switchValue==LOW)
  {
    stepOpen();
  }
  switchValue = digitalRead(STEP_CLOSE);
  if (switchValue==LOW)
  {
    stepClose();
  }
}

void channel_off(void)
{
  analogWrite(CHANNEL_A, 0);
  analogWrite(CHANNEL_B, 0);
}

void screenUp(void)
{
  if (digitalRead(OPEN_BUTTON)==LOW)
    while (current_pos<max_open)
    {
      digitalWrite(MOTOR_BRAKE1, LOW);  //ENABLE CH A
      digitalWrite(MOTOR_BRAKE2, HIGH); //DISABLE CH B
      digitalWrite(MOTOR_DIR1, HIGH);   //Sets direction of CH A
      analogWrite(CHANNEL_A, PWM_SPEED);   //Moves CH A
      delay(50);
      digitalWrite(MOTOR_BRAKE1, HIGH);  //DISABLE CH A
      digitalWrite(MOTOR_BRAKE2, LOW); //ENABLE CH B
      digitalWrite(MOTOR_DIR2, LOW);   //Sets direction of CH B
      analogWrite(CHANNEL_B, PWM_SPEED);   //Moves CH B
      delay(50);
      digitalWrite(MOTOR_BRAKE1, LOW);  //ENABLE CH A
      digitalWrite(MOTOR_BRAKE2, HIGH); //DISABLE CH B
      digitalWrite(MOTOR_DIR1, LOW);   //Sets direction of CH A
      analogWrite(CHANNEL_A, PWM_SPEED);   //Moves CH A
      delay(50);
      digitalWrite(MOTOR_BRAKE1, HIGH);  //DISABLE CH A
      digitalWrite(MOTOR_BRAKE2, LOW); //ENABLE CH B
      digitalWrite(MOTOR_DIR2, HIGH);   //Sets direction of CH B
      analogWrite(CHANNEL_B, PWM_SPEED);   //Moves CH B
      delay(50);
      current_pos+=1;
    }
    channel_off();
}

void screenDown(void)
{
  if (digitalRead(CLOSE_BUTTON)==LOW)
    while (current_pos>min_open)
    {
      digitalWrite(MOTOR_BRAKE1, LOW);  //ENABLE CH A
      digitalWrite(MOTOR_BRAKE2, HIGH); //DISABLE CH B
      digitalWrite(MOTOR_DIR1, HIGH);   //Sets direction of CH A
      analogWrite(CHANNEL_A, PWM_SPEED);   //Moves CH A
      delay(50);
      digitalWrite(MOTOR_BRAKE1, HIGH);  //DISABLE CH A
      digitalWrite(MOTOR_BRAKE2, LOW); //ENABLE CH B
      digitalWrite(MOTOR_DIR2, HIGH);   //Sets direction of CH B
      analogWrite(CHANNEL_B, PWM_SPEED);   //Moves CH B
      delay(50);
      digitalWrite(MOTOR_BRAKE1, LOW);  //ENABLE CH A
      digitalWrite(MOTOR_BRAKE2, HIGH); //DISABLE CH B
      digitalWrite(MOTOR_DIR1, LOW);   //Sets direction of CH A
      analogWrite(CHANNEL_A, PWM_SPEED);   //Moves CH A
      delay(50);
      digitalWrite(MOTOR_BRAKE1, HIGH);  //DISABLE CH A
      digitalWrite(MOTOR_BRAKE2, LOW); //ENABLE CH B
      digitalWrite(MOTOR_DIR2, LOW);   //Sets direction of CH B
      analogWrite(CHANNEL_B, PWM_SPEED);   //Moves CH B
      delay(50);
      current_pos-=1;
    }
  channel_off();
}

void stepOpen(void)
{
  if (digitalRead(STEP_OPEN)==LOW)
    if (current_pos<max_open && limits)
    {
      digitalWrite(MOTOR_BRAKE1, LOW);  //ENABLE CH A
      digitalWrite(MOTOR_BRAKE2, HIGH); //DISABLE CH B
      digitalWrite(MOTOR_DIR1, HIGH);   //Sets direction of CH A
      analogWrite(CHANNEL_A, PWM_SPEED);   //Moves CH A
      delay(50);
      digitalWrite(MOTOR_BRAKE1, HIGH);  //DISABLE CH A
      digitalWrite(MOTOR_BRAKE2, LOW); //ENABLE CH B
      digitalWrite(MOTOR_DIR2, LOW);   //Sets direction of CH B
      analogWrite(CHANNEL_B, PWM_SPEED);   //Moves CH B
      delay(50);
      digitalWrite(MOTOR_BRAKE1, LOW);  //ENABLE CH A
      digitalWrite(MOTOR_BRAKE2, HIGH); //DISABLE CH B
      digitalWrite(MOTOR_DIR1, LOW);   //Sets direction of CH A
      analogWrite(CHANNEL_A, PWM_SPEED);   //Moves CH A
      delay(50);
      digitalWrite(MOTOR_BRAKE1, HIGH);  //DISABLE CH A
      digitalWrite(MOTOR_BRAKE2, LOW); //ENABLE CH B
      digitalWrite(MOTOR_DIR2, HIGH);   //Sets direction of CH B
      analogWrite(CHANNEL_B, PWM_SPEED);   //Moves CH B
      delay(50);
      current_pos+=1;
    }
  channel_off();
}

void stepClose(void)
{
  if (digitalRead(STEP_CLOSE)==LOW)
    if (current_pos>min_open && limits)
    {
      digitalWrite(MOTOR_BRAKE1, LOW);  //ENABLE CH A
      digitalWrite(MOTOR_BRAKE2, HIGH); //DISABLE CH B
      digitalWrite(MOTOR_DIR1, HIGH);   //Sets direction of CH A
      analogWrite(CHANNEL_A, PWM_SPEED);   //Moves CH A
      delay(50);
      digitalWrite(MOTOR_BRAKE1, HIGH);  //DISABLE CH A
      digitalWrite(MOTOR_BRAKE2, LOW); //ENABLE CH B
      digitalWrite(MOTOR_DIR2, HIGH);   //Sets direction of CH B
      analogWrite(CHANNEL_B, PWM_SPEED);   //Moves CH B
      delay(50);
      digitalWrite(MOTOR_BRAKE1, LOW);  //ENABLE CH A
      digitalWrite(MOTOR_BRAKE2, HIGH); //DISABLE CH B
      digitalWrite(MOTOR_DIR1, LOW);   //Sets direction of CH A
      analogWrite(CHANNEL_A, PWM_SPEED);   //Moves CH A
      delay(50);
      digitalWrite(MOTOR_BRAKE1, HIGH);  //DISABLE CH A
      digitalWrite(MOTOR_BRAKE2, LOW); //ENABLE CH B
      digitalWrite(MOTOR_DIR2, LOW);   //Sets direction of CH B
      analogWrite(CHANNEL_B, PWM_SPEED);   //Moves CH B
      delay(50);
      current_pos-=1;
    }
  channel_off();
}


