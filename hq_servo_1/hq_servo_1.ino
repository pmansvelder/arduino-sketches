#include <EEPROM.h>
#include <Servo.h>
#include "EEPROMAnything.h"

// pinning PM+HQ print

const int LedGreen1 = 6;
const int LedGreen2 = 7;
const int LedOrange1 = 4;
const int LedOrange2 = 5;
const int LedRed1 = 2;
const int LedRed2 = 3;
const int Switch1 = 8;
const int Switch2 = 9;
const int OServo1 = 10;
const int OServo2 = 11;
const int Left1 = A0;
const int Left2 = A2;
const int Right1 = A1;
const int Right2 = A3;

// Create Servo objects

Servo servo1;  // create servo object to control a servo
Servo servo2;  // create servo object to control a servo
int value;

// Structure to save values for servos

struct config_t
{
  int min1, min2, max1, max2, speed1, speed2;
} configuration;

void setup()
{

  // Read in configuration from eeprom

  EEPROM_readAnything(0, configuration);

  // Initialize I/O

  pinMode(LedOrange1, OUTPUT);
  pinMode(LedOrange2, OUTPUT);
  pinMode(LedGreen1, OUTPUT);
  pinMode(LedGreen2, OUTPUT);
  pinMode(LedRed1, OUTPUT);
  pinMode(LedRed2, OUTPUT);
  pinMode(Switch1, INPUT);
  pinMode(Switch2, INPUT);
  pinMode(Left1, INPUT);
  pinMode(Right1, INPUT);
  pinMode(Left2, INPUT);
  pinMode(Right2, INPUT);
  servo1.attach(OServo1);
  servo2.attach(OServo2);

  digitalWrite(LedGreen1, HIGH);
  digitalWrite(LedRed1, LOW);
  servo1.write(configuration.min1);

  digitalWrite(LedGreen2, HIGH);
  digitalWrite(LedRed2, LOW);
  servo2.write(configuration.min2);

}

void setupServo1()

{
  digitalWrite(LedOrange1, HIGH);
  int pos = configuration.min1;
  int dir = 1;
  digitalWrite(LedGreen1, HIGH);
  digitalWrite(LedRed1, LOW);
  while (digitalRead(Switch1) == LOW) {}
  while (digitalRead(Switch1) == HIGH) {
    pos += dir;
    servo1.write(pos);
    if (pos > 179) dir = -1;
    if (pos < 1) dir = 1;
    delay(100);
  }
  configuration.min1 = pos;
  pos = configuration.max1;
  dir = -1;
  digitalWrite(LedGreen1, LOW);
  digitalWrite(LedRed1, HIGH);
  while (digitalRead(Switch1) == LOW) {}
  while (digitalRead(Switch1) == HIGH) {
    pos += dir;
    servo1.write(pos);
    if (pos > 179) dir = -1;
    if (pos < 1) dir = 1;
    delay(100);
  }
  configuration.max1 = pos;
  EEPROM_writeAnything(0, configuration);
  delay(300);
  digitalWrite(LedOrange1, LOW);
  digitalWrite(LedGreen1, HIGH);
  digitalWrite(LedRed1, LOW);
  servo1.write(configuration.min1);
}

void setupServo2()
{
  digitalWrite(LedOrange2, HIGH);
  int pos = configuration.min2;
  int dir = 1;
  digitalWrite(LedGreen2, HIGH);
  digitalWrite(LedRed2, LOW);
  while (digitalRead(Switch2) == LOW) {}
  while (digitalRead(Switch2) == HIGH) {
    pos += dir;
    servo2.write(pos);
    if (pos > 179) dir = -1;
    if (pos < 1) dir = 1;
    delay(100);
  }
  configuration.min2 = pos;
  pos = configuration.max2;
  dir = -1;
  digitalWrite(LedGreen2, LOW);
  digitalWrite(LedRed2, HIGH);
  while (digitalRead(Switch2) == LOW) {}
  while (digitalRead(Switch2) == HIGH) {
    pos += dir;
    servo2.write(pos);
    if (pos > 179) dir = -1;
    if (pos < 1) dir = 1;
    delay(100);
  }
  configuration.max2 = pos;
  EEPROM_writeAnything(0, configuration);
  delay(300);
  digitalWrite(LedOrange2, LOW);
  digitalWrite(LedGreen2, HIGH);
  digitalWrite(LedRed2, LOW);
  servo1.write(configuration.min2);
}

void loop()
{
  
// Check for setup buttons

  if (digitalRead(Switch1) == LOW) setupServo1();
  if (digitalRead(Switch2) == LOW) setupServo2();

  if (!digitalRead(Right1)) {
    digitalWrite(LedGreen1, HIGH);
    digitalWrite(LedRed1, LOW);
    servo1.write(configuration.min1);
  }

  if (!digitalRead(Left1)) {
    digitalWrite(LedGreen1, LOW);
    digitalWrite(LedRed1, HIGH);
    servo1.write(configuration.max1);
  }

  if (!digitalRead(Right2)) {
    digitalWrite(LedGreen2, HIGH);
    digitalWrite(LedRed2, LOW);
    servo2.write(configuration.min2);
  }

  if (!digitalRead(Left2)) {
    digitalWrite(LedGreen2, LOW);
    digitalWrite(LedRed2, HIGH);
    servo2.write(configuration.max2);
  }

  delay(100);
}

