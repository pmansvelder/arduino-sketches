// arduino sketch voor de besturing 
// van een navigatiedisplay van een BMW Z4 

#define OPEN_BUTTON 6
#define CLOSE_BUTTON 7
// knoppen open / dicht (rechts)
#define STEP_OPEN 2
#define STEP_CLOSE 4
// knoppen omhoog / omlaag (links)
#define MOTOR_DIR1 12
#define MOTOR_DIR2 13
#define MOTOR_BRAKE1 9
#define MOTOR_BRAKE2 8
#define CHANNEL_A 3
#define CHANNEL_B 11
// pinnen gebruikt voor motorsturing
#define PWM_SPEED 255
// snelheid stappenmotor (255 = max)
#define SETUP_PIN1 5
// pin 5 wordt met een schakelaar naar
// gnd geschakeld voor 'setup mode':
// display kan nu vrij bewegen
#define SETUP_PIN2 10
// pin 10 wordt met een relais naar 
// gnd geschakeld voor signalering
// van 'contact aan / uit'
#define MAX_STEPS 12
// Aantal stappen van dicht naar open

int min_open = 0;
int max_open = MAX_STEPS;
int current_pos = -1;
boolean limits = true;
boolean action = true;

void setup()
{
  // initialiseer variabelen
  pinMode(SETUP_PIN1, INPUT_PULLUP);
  if (digitalRead(SETUP_PIN1)==LOW)
  {
    limits = false;
  }
  current_pos = min_open;
  pinMode(SETUP_PIN2, INPUT_PULLUP);
  // stel schakelpinnen in
  pinMode(OPEN_BUTTON, INPUT_PULLUP);
  pinMode(CLOSE_BUTTON, INPUT_PULLUP);
  pinMode(STEP_OPEN, INPUT_PULLUP);
  pinMode(STEP_CLOSE, INPUT_PULLUP);
  // stel motor richtingspinnen in
  pinMode(MOTOR_DIR1, OUTPUT);
  pinMode(MOTOR_DIR2, OUTPUT);
  // stel motor rempinnen in
  pinMode(MOTOR_BRAKE1, OUTPUT);
  pinMode(MOTOR_BRAKE2, OUTPUT);
}

void loop(void)
{
  int switchValue;
  switchValue = digitalRead(SETUP_PIN2);
  // SETUP_PIN2 must be connected to a optocoupler and ground, to a sense voltage that is high when contact is made
  if ((switchValue==LOW) && action)
  {
    screenUp();
    action = false;
  }
  if ((switchValue==HIGH) && !action)
  { 
    screenDown();
    action = true;
  }
  switchValue = digitalRead(OPEN_BUTTON);
  if (switchValue==LOW)
  {
    if (digitalRead(OPEN_BUTTON)==LOW)
      screenUp();
  }
  switchValue = digitalRead(CLOSE_BUTTON);
  if (switchValue==LOW)
  {
    if (digitalRead(CLOSE_BUTTON)==LOW)
      screenDown();
  }
  switchValue = digitalRead(STEP_OPEN);
  if (switchValue==LOW)
  {
    if (digitalRead(STEP_OPEN)==LOW)
      stepOpen();
  }
  switchValue = digitalRead(STEP_CLOSE);
  if (switchValue==LOW)
  {
    if (digitalRead(STEP_CLOSE)==LOW)
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
    while (current_pos<=max_open)
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
    while (current_pos>=min_open)
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
  if ((current_pos<=max_open) || !limits)
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
  if ((current_pos>=min_open) || !limits)
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


