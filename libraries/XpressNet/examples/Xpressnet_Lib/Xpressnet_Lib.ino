/*
  XpressNet for Arduino
  
  Showing the funktion of the Library
  
  use for Arduino Mega Serial 1; for Arduino UNO Serial
  
*/

//----------------------------------------------------------------------------
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define SerialDEBUG 1  //For Serial Port Debugging (Arduino Mega only)
#endif  

//----------------------------------------------------------------------------

#include <XpressNet.h> 
XpressNetClass XpressNet;

//Debug Ausgabe
#include <SoftwareSerial.h>
SoftwareSerial Debug(5, 6); // RX, TX

// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;

//--------------------------------------------------------------
// XpressNet address: must be in range of 1-31; must be unique. Note that some IDs
// are currently used by default, like 2 for a LH90 or LH100 out of the box, or 30
// for PC interface devices like the XnTCP.
#define XNetAddress 30    //Adresse im XpressNet
#define XNetSRPin 5       //Max485 Busdriver Send/Receive-PIN

long previousMillisA = 0;        // will store last time LED was updated
#define interval 8500 

int valA = 0;
int valB = 1;
int valC = 0;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
 digitalWrite(led, HIGH); 
 delay(1000);
  
  #if SerialDEBUG
  Debug.begin(115200);
  Debug.println("XNet Test");
  #endif
 
  XpressNet.start(XNetAddress, XNetSRPin);    //Initialisierung XNet-Bus

  XpressNet.setPower(csNormal);
  #if SerialDEBUG
  notifyXNetPower (XpressNet.getPower());
  #endif
}

// the loop routine runs over and over again forever:
void loop() {
  
  XpressNet.receive();
  
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillisA > interval) {
    previousMillisA = currentMillis;
    if (valA != 0) 
      valA = 0;
    else valA = 1;
    XpressNet.setLocoFunc(0x00, 22, valA, 0);   //2 = umschalten
    #if SerialDEBUG
    Debug.print(valA);
    Debug.println(" ChangeFunc 22");
    #endif
    if (valB != 0) 
      valB = 0;
    else valB = 1;  
    XpressNet.setLocoFunc(0x00, 11, valB, 0);   //2 = umschalten
    #if SerialDEBUG
    Debug.print(valB);
    Debug.println(" ChangeFunc 11");
    #endif
    if (valC != 0) 
      valC = 0;
    else valC = 1;
    XpressNet.setLocoFunc(0x00, 5, valC, 0);   //2 = umschalten
    #if SerialDEBUG
    Debug.print(valC);
    Debug.println(" ChangeFunc 5");
    Debug.println("WAIT");
    #endif
    
/*    
    XpressNet.setHalt();			//Zustand Halt Melden
    XpressNet.getLocoInfo (byte Adr_High, byte Adr_Low);	//Abfragen der Lokdaten (mit F0 bis F12)
    XpressNet.getLocoFunc (byte Adr_High, byte Adr_Low);	//Abfragen der Lok Funktionszustände F13 bis F28
    XpressNet.setLocoHalt (byte Adr_High, byte Adr_Low);	//Lok anhalten
    
    //Steps = 0x02 (14,27,28,128);  Bit7 of Speed = Direction
    XpressNet.setLocoDrive (byte Adr_High, byte Adr_Low, uint8_t Steps, uint8_t Speed); //Lokdaten setzten
    
    //type: 0 = off, 1 = on, 2 = change;  fkt: 0..28 (the number of the function to change)
    XpressNet.setLocoFunc (byte Adr_High, byte Adr_Low, uint8_t type, uint8_t fkt);	//Lokfunktion setzten
    XpressNet.getLocoStateFull (byte Adr_High, byte Adr_Low, false);  //Gibt Zustand der Lok zurück.
    XpressNet.getTrntInfo (byte FAdr_High, byte FAdr_Low);		//Ermitteln der Schaltstellung einer Weiche
    XpressNet.setTrntPos (byte FAdr_High, byte FAdr_Low, byte Pos);		//Schalten einer Weiche
	//Programming:
    XpressNet.readCVMode (byte CV);	//Lesen der CV im CV-Mode
    XpressNet.writeCVMode (byte CV, byte Data);		//Schreiben einer CV im CV-Mode
    XpressNet.getresultCV();		//Programmierergebnis anfordern
*/    
  }
  
}

void notifyXNetDebug(String s) 
{
  #if SerialDEBUG
  Debug.println(s);
  #endif
}

//--------------------------------------------------------------------------------------------
void notifyXNetStatus (uint8_t State)
{
  digitalWrite(led, State);
}

//--------------------------------------------------------------------------------------------
void notifyXNetPower (uint8_t State)
{
  #if SerialDEBUG
  Debug.print("Power: ");
  switch (State) {
    case csNormal: Debug.println("ON"); break;
    case csTrackVoltageOff: Debug.println("OFF"); break;
    case csEmergencyStop: Debug.println("EmStop"); break;
    case csShortCircuit: Debug.println("SHORT"); break;
    case csServiceMode: Debug.println("PROG"); break;
  }
  #endif
}

//--------------------------------------------------------------------------------------------
void notifyXNetVersion(uint8_t Version, uint8_t ID ) 
{
  #if SerialDEBUG
  Debug.print("Version: ");
  Debug.println(Version, HEX);
  #endif
}

//--------------------------------------------------------------------------------------------
void notifyLokAll(uint8_t Adr_High, uint8_t Adr_Low, boolean Busy, uint8_t Steps, uint8_t Speed, uint8_t Direction, uint8_t F0, uint8_t F1, uint8_t F2, uint8_t F3, boolean Req ) {
  #if SerialDEBUG
   Debug.print(Busy); 
   Debug.print(" Loco ALL: "); 
   Debug.print(Adr_Low); 
   Debug.print(", Stufen: "); 
   Debug.print(Steps, BIN); 
   Debug.print(", Speed: "); 
   Debug.print(Speed); 
   Debug.print(", Richtung: "); 
   Debug.print(Direction); 
   Debug.print(", Fkt: "); 
   Debug.print(F0, BIN); 
   Debug.print("; "); 
   Debug.print(F1, BIN); 
   Debug.print(", Fkt2: "); 
   Debug.print(F2, BIN); 
   Debug.print("; "); 
   Debug.println(F3, BIN); 
  #endif 
}

//--------------------------------------------------------------------------------------------
void notifyLokFunc(uint8_t Adr_High, uint8_t Adr_Low, uint8_t F2, uint8_t F3 ) {
  #if SerialDEBUG
  Debug.print("Loco Fkt: "); 
  Debug.print(Adr_Low); 
  Debug.print(", Fkt2: "); 
  Debug.print(F2, BIN); 
  Debug.print("; "); 
  Debug.println(F3, BIN); 
  #endif
}

//--------------------------------------------------------------------------------------------
void notifyTrnt(uint8_t Adr_High, uint8_t Adr_Low, uint8_t Pos) {
  #if SerialDEBUG
  Debug.print("Weiche: "); 
  Debug.print(word(Adr_High, Adr_Low)); 
  Debug.print(", Position: "); 
  Debug.println(Pos, BIN); 
  #endif
}

//--------------------------------------------------------------------------------------------
void notifyCVInfo(uint8_t State ) {
  #if SerialDEBUG
  Debug.print("CV Prog STATE: "); 
  Debug.println(State); 
  #endif
}

//--------------------------------------------------------------------------------------------
void notifyCVResult(uint8_t cvAdr, uint8_t cvData ) {
  #if SerialDEBUG
  Debug.print("CV Prog Read: "); 
  Debug.print(cvAdr); 
  Debug.print(", "); 
  Debug.println(cvData); 
  #endif
}
