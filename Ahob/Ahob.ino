// Program to simulate a dutch railway crossing
// 2 contacts, can be any sensor that goes low when a train is passing
// (current detector, ledgate, hall sensor, Maerklin ground contact)
// when first contact is triggered, 2 leds start blinking alternately
// and a servo is triggered (as yet unimplemented)
// this continues until second contact is triggered and a fixed delay has passed,
// then the leds are extinguished, the servo is reset,
// and the loop is reset, and waits for another round.

#include <Servo.h> 

// constants

const int contact1 = 2;       // contact 1 and 2 for entry and exit of railway crossing
const int contact2 = 3;
const int wacht = 200;        // blinking delay
const int uitloop = 500;      // variable for delay after second contact is triggered
const int knipper1 = 12;      // outputs for blinking lights
const int knipper2 = 13;
const int omlaag1 = 90;
const int omlaag2 = 90;
int stap = 1;
int stap_delay;

int buttonState1 = 0;         // variable for reading the pushbutton status
int buttonState2 = 0;         // variable for reading the pushbutton status
int flipflop = 0;             // variable for status of blinking
unsigned long teller = 0;               // variable to keep track of time passed since exit contact
int exitsensor = 0;           // variable for exit sensor
int start;
int eind;
int pos;
int servo_pos;
int servo1_output = 9;        // output for servo 1
int servo2_output = 10;       // output for servo 2

Servo servo1;
Servo servo2;

// the setup routine runs once when you press reset:
void setup() {            
  Serial.begin(19200);
  // initialize the digital pin as an output.
  pinMode(knipper1, OUTPUT);
  pinMode(knipper2, OUTPUT);  
  pinMode(contact1, INPUT);
  pinMode(contact2, INPUT);
  digitalWrite(contact1, HIGH);
  digitalWrite(contact2, HIGH);
  // initialize servos
  servo1.attach(servo1_output);
  servo2.attach(servo2_output);
  servo1.write(0);              
  servo2.write(0);
}

void knipper() {
  digitalWrite(knipper1, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(knipper2, LOW);
  delay(wacht);               // wait for a second
  digitalWrite(knipper1, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(knipper2, HIGH);
  delay(wacht);               // wait for a second
}

void SetServos() {
  stap_delay = 10;
  start = servo_pos;
  if (eind > start) {
    Serial.print("forward");
    Serial.println(eind);
    for (pos = start; pos < eind; pos += stap)   
      {                                  
        servo1.write(pos);              // tell servo to go to position in variable 'pos'
        servo2.write(pos);
        delay(stap_delay);              // waits for the servo to reach the position 
    }
  }
  else 
  if (start > eind) {
     Serial.print("backwards");
     Serial.println(eind);
     for (pos = start; pos > eind; pos -= stap)  // goes from 0 degrees to 180 degrees 
      {                                  // in steps of 1 degree 
        servo1.write(pos);              // tell servo to go to position in variable 'pos'
        servo2.write(pos);
        delay(stap_delay);                       // waits for the servo to reach the position 
      }
    }
    servo_pos = eind;
  }

void loop(){
  
  // a new loop, all sensors cleared, flipflop triggered
  exitsensor = 0;
  teller = 0;
  flipflop = 1;
  
  // servos to first position
  
  servo_pos = 0;
  eind = 0;
  SetServos();
    
  // read the state of the contacts:

  buttonState1 = !digitalRead(contact1);
  buttonState2 = !digitalRead(contact2);

  // now determine if any contact is high:
  
  if (buttonState1 == HIGH) {
    exitsensor = contact2;
  }
  if (buttonState2 == HIGH) {
    exitsensor = contact1;
  }

  // if one contact was triggered, exitsensor has a value != 0
  // and the loop begins

  while (exitsensor != 0) {
    // set the servo (TODO)
    if (servo_pos != omlaag1) {
    eind = omlaag1;
    SetServos();
    }
    knipper();
    // the loop ends when the 'other' contact is trigered
    buttonState1 = !digitalRead(exitsensor);
    if (buttonState1 == HIGH) {
      Serial.println("Second sensor triggered");
    // stop blinking, but only after a delay:
      flipflop = 0;
      teller = millis();
      // reset the servos (TODO)
      eind = 0;
      SetServos();
    }
    if (flipflop == 0) {
    while ((millis() - teller) < uitloop) {
      knipper();
      }
    digitalWrite(knipper1, LOW); 
    digitalWrite(knipper2, LOW);
    exitsensor = 0;
    }
  }
}
