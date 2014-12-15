/*
  Button
 
 Turns on and off a light emitting diode(LED) connected to digital  
 pin 13, when pressing a pushbutton attached to pin 2. 
 
 
 The circuit:
 * LED attached from pin 13 to ground 
 * pushbutton attached to pin 2 from +5V
 * 10K resistor attached to pin 2 from ground
 
 * Note: on most Arduinos there is already an LED on the board
 attached to pin 13.
 
 */

// constants won't change. They're used here to 
// set pin numbers:

const int sensor1 = 0;
const int sensor2 = 1;

const int ledGreen = 8;
const int ledYellow = 9;
const int ledRed = 10;


// variables will change:
int sensor1State = 0;
int sensor2State = 0;

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledGreen, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(ledRed, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(sensor1, INPUT);
  digitalWrite(sensor1, HIGH);
  pinMode(sensor2, INPUT);
  digitalWrite(sensor2, HIGH);
}

void loop(){
  // read the state of the pushbutton value:
  sensor1State = !digitalRead(sensor1);
  sensor2State = !digitalRead(sensor2);
  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (sensor1State == HIGH) {     
    // turn LED on:    
    digitalWrite(ledRed, HIGH);  
    digitalWrite(ledYellow, LOW);  
    digitalWrite(ledGreen, LOW);  
  } 
  else {
    if (sensor2State == HIGH) {
      digitalWrite(ledRed, LOW);  
      digitalWrite(ledYellow, HIGH);  
      digitalWrite(ledGreen, LOW);
    }
    else {
      digitalWrite(ledRed, LOW);  
      digitalWrite(ledYellow, LOW);  
      digitalWrite(ledGreen, HIGH);  
    }
  }
}
