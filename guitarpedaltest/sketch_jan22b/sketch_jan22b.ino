#include "dsp.h"

// create an array for the delay
byte array[2000]; 

//define variables
int j;
int fx;
int mode;
int value50;
int value300;
int value10000;
int delayed;

void setup() {
  setupIO();

   Serial.begin(9600);
  
  //set initial values
  j = 50;
  value50 = 50;
  value300 = 300;
  value10000 = 1000;
  delayed = 0;
}

void readKnobs(){
  //read the rotary switch
  //and determine which effect is selected
  //dividing by 75 ensures proper discrete values
  //for if statements above
  mode = analogRead(2);
  mode = mode / 75;
    
  //reads the effects pot to adjust
  //the intensity of the effects above  
  fx = analogRead(3);

}

void loop() {
  
    //check status of the effect potentiometer and rotary switch
    readKnobs();
    Serial.println(mode);
    delay(2000);
    
}


