// ARDUINORAILMAKET.RU
// V.4.1
// 01.04.2017
// Author: Steve Massikker


/////// SIMPLE COMMAND STATION ///////

#include <SoftwareSerial.h>
#include <Servo.h>

//// GPIO PINS ////

// SOFTWARE SERIAL
SoftwareSerial Bluetooth(12, 13); // RX, TX

// L298
#define ENA_PIN 6
#define IN1_PIN 7
#define IN2_PIN 5
#define IN3_PIN 2
#define IN4_PIN 4
#define ENB_PIN 3

// RELAY
#define RELAY_IN1 14
#define RELAY_IN2 15

// SIGNALS
#define CHANNEL_1 19
#define CHANNEL_2 18
#define CHANNEL_3 17
#define CHANNEL_4 16

// JUNCTIONS
#define JUNCTION_EN 8 
#define JUNCTION1_PIN 9
Servo JNC1;
#define JUNCTION2_PIN 10
Servo JNC2; 
#define JUNCTION3_PIN 11
Servo JNC3; 


//// VARIABLES ////
boolean stringComplete = false;
String inputString = ""; 
unsigned long millisJunction;


void setup() {

// Initialize Serial
  Serial.begin(9600);
  Bluetooth.begin(9600);   
  inputString.reserve(16); 

// Initialize Motor Driver
  pinMode(ENA_PIN, OUTPUT); 
  pinMode(IN1_PIN, OUTPUT); 
  pinMode(IN2_PIN, OUTPUT); 
  pinMode(IN3_PIN, OUTPUT); 
  pinMode(IN4_PIN, OUTPUT); 
  pinMode(ENB_PIN, OUTPUT);  

// Initialize relay & signals
  pinMode(RELAY_IN1, OUTPUT); 
  pinMode(RELAY_IN2, OUTPUT); 
  pinMode(CHANNEL_1, OUTPUT); 
  pinMode(CHANNEL_2, OUTPUT); 
  pinMode(CHANNEL_3, OUTPUT); 
  pinMode(CHANNEL_4, OUTPUT); 

// Initialize Servos
  pinMode(JUNCTION1_PIN, OUTPUT); 
  JNC1.attach(JUNCTION1_PIN);
  pinMode(JUNCTION2_PIN, OUTPUT); 
  JNC2.attach(JUNCTION2_PIN);
  pinMode(JUNCTION3_PIN, OUTPUT); 
  JNC3.attach(JUNCTION3_PIN);  
  pinMode(JUNCTION_EN, OUTPUT);

//// STARTUP ////   

// Junction
  digitalWrite(JUNCTION_EN, HIGH);
  delay(80);
  JNC1.write(0);
  delay(100);  
  JNC2.write(0);
  delay(100);  
  JNC3.write(180); 
  delay(800);
  digitalWrite(JUNCTION_EN, LOW);  

// Realy & Signal 
  digitalWrite(RELAY_IN1, HIGH);
  digitalWrite(RELAY_IN2, HIGH);
  digitalWrite(CHANNEL_1, LOW);
  digitalWrite(CHANNEL_2, HIGH);
  digitalWrite(CHANNEL_3, LOW);
  digitalWrite(CHANNEL_4, HIGH); 

// L298 N (optional)
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(ENB_PIN, LOW);

}

void loop() {

  if (stringComplete) {
    Serial.print("inputString = ");    // PRINT COMMAND //
    Serial.println(inputString);       // TO SERIAL     //

// ----------- START COMMAND PARSING ----------- // 

    //THROTTLE LOOP TRACK
    if (inputString.charAt(0) =='t') {   
      if (inputString.charAt(1) =='0') {
        analogWrite(ENB_PIN, 0);
      }
      if (inputString.charAt(1) =='1') { 
        analogWrite(ENB_PIN, 80);
      }
      if (inputString.charAt(1) =='2') {
        analogWrite(ENB_PIN, 100);  // START LOCO
      }
      if (inputString.charAt(1) =='3') { 
        analogWrite(ENB_PIN, 150);
      }
      if (inputString.charAt(1) =='4') {
        analogWrite(ENB_PIN, 200);
      }    
      if (inputString.charAt(1) =='5') { 
        analogWrite(ENB_PIN, 255);
      } 
    }

    // DIRECTION LOOP TRACK
    if (inputString.charAt(0) =='d') {
      if (inputString.charAt(1) =='r') {
        digitalWrite(IN3_PIN, HIGH);
        digitalWrite(IN4_PIN, LOW);
      }
      if (inputString.charAt(1) =='f') {
        digitalWrite(IN3_PIN, LOW);
        digitalWrite(IN4_PIN, HIGH);
      }
      if (inputString.charAt(1) =='s') {
        digitalWrite(IN3_PIN, LOW);
        digitalWrite(IN4_PIN, LOW);
        analogWrite(ENB_PIN, 0);
      } 
    }
      
    //THROTTLE TEST TRACK
    if (inputString.charAt(0) =='u') {   
      if (inputString.charAt(1) =='0') {
        analogWrite(ENA_PIN, 0);
      }
      if (inputString.charAt(1) =='a') { 
        analogWrite(ENA_PIN, 60);
      }
      if (inputString.charAt(1) =='b') {
        analogWrite(ENA_PIN, 90);  
      }
      if (inputString.charAt(1) =='c') { 
        analogWrite(ENA_PIN, 120);
      }
      if (inputString.charAt(1) =='d') {
        analogWrite(ENA_PIN, 150);
      }    
      if (inputString.charAt(1) =='e') { 
        analogWrite(ENA_PIN, 180);
      } 
      if (inputString.charAt(1) =='f') { 
        analogWrite(ENA_PIN, 210);
      } 
      if (inputString.charAt(1) =='g') { 
        analogWrite(ENA_PIN, 240);
      } 
      if (inputString.charAt(1) =='h') { 
        analogWrite(ENA_PIN, 255);
      }       
    }

    // DIRECTION TEST TRACK
    if (inputString.charAt(0) =='d') {
      if (inputString.charAt(1) =='n') {
        digitalWrite(IN1_PIN, HIGH);
        digitalWrite(IN2_PIN, LOW);
      }
      if (inputString.charAt(1) =='v') {
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, HIGH);
      }
      if (inputString.charAt(1) =='o') {
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, LOW);
        analogWrite(ENA_PIN, 0);
      } 
    }

    //JUNCTIONS  
    if (inputString.charAt(0) =='j') { 
      if (inputString.charAt(1) =='a') { 
        digitalWrite(JUNCTION_EN, HIGH);
        JNC1.write(0);
        delay(100); // servo start current limitation
        JNC2.write(0);        
        millisJunction = millis();  
        digitalWrite(CHANNEL_1, LOW);        
        digitalWrite(CHANNEL_2, HIGH);
        delay(100);
        digitalWrite(RELAY_IN1, HIGH);                
      }
      if (inputString.charAt(1) =='b') { 
        digitalWrite(JUNCTION_EN, HIGH);
        JNC1.write(180);
        delay(100); // servo on start current limitation
        JNC2.write(180);        
        millisJunction = millis();  
        digitalWrite(CHANNEL_1, HIGH);        
        digitalWrite(CHANNEL_2, LOW);
        delay(100);
        digitalWrite(RELAY_IN1, LOW);      
      }
      if (inputString.charAt(1) =='c') { 
        digitalWrite(JUNCTION_EN, HIGH);
        JNC3.write(180);
        millisJunction = millis();  
        digitalWrite(CHANNEL_3, HIGH);        
        digitalWrite(CHANNEL_4, LOW);
        delay(100);
        digitalWrite(RELAY_IN2, HIGH);      
      } 
      if (inputString.charAt(1) =='d') { 
        digitalWrite(JUNCTION_EN, HIGH);
        JNC3.write(0);
        millisJunction = millis();  
        digitalWrite(CHANNEL_3, LOW);        
        digitalWrite(CHANNEL_4, HIGH);
        delay(100);
        digitalWrite(RELAY_IN2, LOW);      
      }  
    }

// ----------- END COMMAND PARSING ----------- // 

    inputString = "";
    stringComplete = false;    
  }

  bluetoothEvent(); 
  if (millis() > (millisJunction + 800)) digitalWrite(JUNCTION_EN, LOW); // off power servo

}

// ----------- FUNCTIONS ----------- // 

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == 'z') {
      stringComplete = true;
    }
  }
}

void bluetoothEvent() {
  while (Bluetooth.available()) {
    char inChar = (char)Bluetooth.read();
    inputString += inChar;
    if (inChar == 'z') {
      stringComplete = true;
    }
  }
}
