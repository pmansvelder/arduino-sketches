/*
  XpressNet for Arduino
  
  Showing the funktion of the Library for a Railpower OFF Button

  The circuit:
 * LED attached from pin 13 to ground and from pin 12 to ground
 * pushbutton attached to pin 2 from +5V
 * 10K resistor attached to pin 2 from ground
 
 * * Note: on most Arduinos there is already an LED on the board
     attached to pin 13.
  
*/

#include <XpressNet.h> 
XpressNetClass XpressNet;

// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
byte LedPin = 13;

byte PowerLedPin = 12;  //Signalisiert den Gleisspannungszustand

byte ButtonPin = 2;     //Taster zum Abschalten der Gleisspannung

byte lastButtonState = HIGH;    // variable for reading the Button status

//--------------------------------------------------------------
// XpressNet address: must be in range of 1-31; must be unique. Note that some IDs
// are currently used by default, like 2 for a LH90 or LH100 out of the box, or 30
// for PC interface devices like the XnTCP.
#define XNetAddress 30    //Adresse im XpressNet
#define XNetSRPin 5       //Max485 Busdriver Send/Receive-PIN

//--------------------------------------------------------------------------------------------
// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(LedPin, OUTPUT);
  digitalWrite(LedPin, HIGH); 
  
  pinMode(PowerLedPin, OUTPUT);
  digitalWrite(PowerLedPin, LOW);

  pinMode(ButtonPin, INPUT);
  digitalWrite(ButtonPin, HIGH);  //Pull_Up
 
  XpressNet.start(XNetAddress, XNetSRPin);    //Initialisierung XNet-Bus
}

//--------------------------------------------------------------------------------------------
// the loop routine runs over and over again forever:
void loop() {
  XpressNet.receive();  //permernet update the library
  
  CheckButton();  //check if Button push
}

//--------------------------------------------------------------------------------------------
void CheckButton() 
{
  // read the state of the pushbutton value:
  if (digitalRead(ButtonPin) == LOW && lastButtonState == HIGH) { //down
    lastButtonState = LOW;  //save state
    if (XpressNet.getPower() != csTrackVoltageOff)  //just send when State is not Power OFF
      XpressNet.setPower(csTrackVoltageOff);
  }
  if (digitalRead(ButtonPin) == HIGH && lastButtonState == LOW) { //up
    lastButtonState = HIGH; //save state
  }

}


//--------------------------------------------------------------------------------------------
void notifyXNetStatus (uint8_t State)
{
  digitalWrite(LedPin, State);
}

//--------------------------------------------------------------------------------------------
void notifyXNetPower (uint8_t State)
{
  switch (State) {
    case csNormal: digitalWrite(PowerLedPin, HIGH); break;
    case csTrackVoltageOff: digitalWrite(PowerLedPin, LOW); break;
    case csEmergencyStop: digitalWrite(PowerLedPin, HIGH); break;
    case csShortCircuit: digitalWrite(PowerLedPin, LOW); break;
    case csServiceMode: digitalWrite(PowerLedPin, LOW); break;
  }
}
