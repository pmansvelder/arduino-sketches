// program to move the navigation display 
// on a BMW Z4 using 4 buttons


#include <EEPROM.h>
#define CONFIG_VERSION "bmw"
#define CONFIG_START 32

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
#define SETUP_PIN2 10
#define MAX_STEPS 50

int min_open = 0;
int max_open;
int current_pos = 200;
bool ok = false;

struct StoreStruct {
  // This is for mere detection if they are your settings
  char version[4];
  // The variables of your settings
  int pos;
} storage = {
  CONFIG_VERSION,
  // The default values
  0
};

void loadConfig() {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2])
    for (unsigned int t=0; t<sizeof(storage); t++)
      *((char*)&storage + t) = EEPROM.read(CONFIG_START + t);
}

void saveConfig() {
  for (unsigned int t=0; t<sizeof(storage); t++)
    EEPROM.write(CONFIG_START + t, *((char*)&storage + t));
}

void setup()
{
  loadConfig();
  max_open = min_open + MAX_STEPS;
  pinMode(SETUP_PIN1, INPUT_PULLUP);
  if (SETUP_PIN1==LOW)
  {
    //initialize position
    current_pos = min_open;
    saveConfig();
  }
  pinMode(SETUP_PIN2, INPUT_PULLUP);
  if (SETUP_PIN2==LOW)
  {
    //initialize position
    current_pos = max_open;
    saveConfig();
  }
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
  int current_pos = storage.pos;
  int switchValue;
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
  storage.pos = current_pos;
  if (ok)
    {
    saveConfig();
    ok = false;
    }
}

void channel_off(void)
{
  analogWrite(CHANNEL_A, 0);
  analogWrite(CHANNEL_B, 0);
  ok = true;
}

void screenUp(void)
{
  if (digitalRead(OPEN_BUTTON)==LOW)
    if (current_pos<max_open)
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
      channel_off();
    }
}

void screenDown(void)
{
  if (digitalRead(CLOSE_BUTTON)==LOW)
    if (current_pos>min_open)
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
      channel_off();
    }
}

void stepOpen(void)
{
  if (digitalRead(STEP_OPEN)==LOW)
    digitalWrite(MOTOR_BRAKE1, LOW);  //ENABLE CH A
    digitalWrite(MOTOR_BRAKE2, HIGH); //DISABLE CH B
    digitalWrite(MOTOR_DIR1, HIGH);   //Sets direction of CH A
    analogWrite(CHANNEL_A, PWM_SPEED);   //Moves CH A
    delay(1000);
    digitalWrite(MOTOR_BRAKE1, HIGH);  //DISABLE CH A
    digitalWrite(MOTOR_BRAKE2, LOW); //ENABLE CH B
    digitalWrite(MOTOR_DIR2, LOW);   //Sets direction of CH B
    analogWrite(CHANNEL_B, PWM_SPEED);   //Moves CH B
    delay(1000);
    digitalWrite(MOTOR_BRAKE1, LOW);  //ENABLE CH A
    digitalWrite(MOTOR_BRAKE2, HIGH); //DISABLE CH B
    digitalWrite(MOTOR_DIR1, LOW);   //Sets direction of CH A
    analogWrite(CHANNEL_A, PWM_SPEED);   //Moves CH A
    delay(1000);
    digitalWrite(MOTOR_BRAKE1, HIGH);  //DISABLE CH A
    digitalWrite(MOTOR_BRAKE2, LOW); //ENABLE CH B
    digitalWrite(MOTOR_DIR2, HIGH);   //Sets direction of CH B
    analogWrite(CHANNEL_B, PWM_SPEED);   //Moves CH B
    delay(1000);
    channel_off();
}

void stepClose(void)
{
  if (digitalRead(STEP_CLOSE)==LOW)
    digitalWrite(MOTOR_BRAKE1, LOW);  //ENABLE CH A
    digitalWrite(MOTOR_BRAKE2, HIGH); //DISABLE CH B
    digitalWrite(MOTOR_DIR1, HIGH);   //Sets direction of CH A
    analogWrite(CHANNEL_A, PWM_SPEED);   //Moves CH A
    delay(1000);
    digitalWrite(MOTOR_BRAKE1, HIGH);  //DISABLE CH A
    digitalWrite(MOTOR_BRAKE2, LOW); //ENABLE CH B
    digitalWrite(MOTOR_DIR2, HIGH);   //Sets direction of CH B
    analogWrite(CHANNEL_B, PWM_SPEED);   //Moves CH B
    delay(1000);
    digitalWrite(MOTOR_BRAKE1, LOW);  //ENABLE CH A
    digitalWrite(MOTOR_BRAKE2, HIGH); //DISABLE CH B
    digitalWrite(MOTOR_DIR1, LOW);   //Sets direction of CH A
    analogWrite(CHANNEL_A, PWM_SPEED);   //Moves CH A
    delay(1000);
    digitalWrite(MOTOR_BRAKE1, HIGH);  //DISABLE CH A
    digitalWrite(MOTOR_BRAKE2, LOW); //ENABLE CH B
    digitalWrite(MOTOR_DIR2, LOW);   //Sets direction of CH B
    analogWrite(CHANNEL_B, PWM_SPEED);   //Moves CH B
    delay(1000);
    channel_off();
}


