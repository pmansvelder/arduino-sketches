/*
  XpressNet for Arduino
  
  Showing the funktion of the Library for a Switch control
  The circuit:
 * LED and resistor attached from pin 13 to ground 
 * LED and resistor attached from pin 12 to ground (switch staight)
 * LED and resistor attached from pin 12 to +5V (switch turn)
 * pushbutton attached to pin 2 from +5V and 10K resistor attached to pin 2 from ground
 
 * * Note: on most Arduinos there is already an LED on the board
     attached to pin 13.
  
*/

#include <XpressNet.h> 
XpressNetClass XpressNet;

// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
byte LedPin = 13;

byte SwitchLedPin = 12;  //Signalisiert den Weichenzustand

byte ButtonSPin = 2;     //Taster zum Abschalten der Gleisspannung
byte ButtonTurnPin = 3;     //Taster zum Abschalten der Gleisspannung

byte lastButtonSState = HIGH;    // variable for reading the Button status
byte lastButtonTurnState = HIGH;    // variable for reading the Button status

unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 1000;           // interval at which to blink (milliseconds)
//--------------------------------------------------------------
// XpressNet address: must be in range of 1-31; must be unique. Note that some IDs
// are currently used by default, like 2 for a LH90 or LH100 out of the box, or 30
// for PC interface devices like the XnTCP.
#define XNetAddress 30    //Adresse im XpressNet
#define XNetSRPin 5       //Max485 Busdriver Send/Receive-PIN

#define SwitchAdr 18      //Adresse der Beispielweiche

//--------------------------------------------------------------------------------------------
// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(LedPin, OUTPUT);
  digitalWrite(LedPin, HIGH); 
  
  pinMode(SwitchLedPin, OUTPUT);
  digitalWrite(SwitchLedPin, LOW);

  pinMode(ButtonSPin, INPUT);
  digitalWrite(ButtonSPin, HIGH);  //Pull_Up
  pinMode(ButtonTurnPin, INPUT);
  digitalWrite(ButtonTurnPin, HIGH);  //Pull_Up
 
  XpressNet.start(XNetAddress, XNetSRPin);    //Initialisierung XNet-Bus
}

//--------------------------------------------------------------------------------------------
// the loop routine runs over and over again forever:
void loop() {
  XpressNet.receive();  //permernet update the library
  
  CheckButton();  //check if Button push

  //Ask for switch state
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED 
    previousMillis = currentMillis;  
    XpressNet.getTrntInfo(0, SwitchAdr);
  }
}

//--------------------------------------------------------------------------------------------
void CheckButton() 
{
  // read the state of the pushbutton value:
  //Gerade
  if (digitalRead(ButtonSPin) == LOW && lastButtonSState == HIGH) { //down
    lastButtonSState = LOW;  //save state
    digitalWrite(SwitchLedPin, HIGH);
    
    XpressNet.setTrntPos(0, SwitchAdr, B1000); //switch 18 gerade
  }
  if (digitalRead(ButtonSPin) == HIGH && lastButtonSState == LOW) { //up
    lastButtonSState = HIGH; //save state
  }
  //Abzweigen
  if (digitalRead(ButtonTurnPin) == LOW && lastButtonTurnState == HIGH) { //down
    lastButtonTurnState = LOW;  //save state
    digitalWrite(SwitchLedPin, LOW);
    
    XpressNet.setTrntPos(0, SwitchAdr, B0000); //switch 18 turn
  }
  if (digitalRead(ButtonTurnPin) == HIGH && lastButtonTurnState == LOW) { //up
    lastButtonTurnState = HIGH; //save state
  }

}

//--------------------------------------------------------------------------------------------
void notifyTrnt(uint8_t Adr_High, uint8_t Adr_Low, uint8_t Pos)
{
  if (Adr_Low == SwitchAdr) {
    if (Pos == B10)
      digitalWrite(SwitchLedPin, LOW);
    if (Pos == B01)  
      digitalWrite(SwitchLedPin, HIGH);  
  }
}


//--------------------------------------------------------------------------------------------
void notifyXNetStatus (uint8_t State)
{
  digitalWrite(LedPin, State);
}
