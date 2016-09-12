/*
* Creates a minimum DCC command station 
 */
 
#include <EEPROM.h>
 
#include <p50x.h> 
#include <DCCPacket.h>
#include <DCCPacketQueue.h>
#include <DCCPacketScheduler.h>

DCCPacketScheduler dcc;

p50xClass p50x;

// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;
byte cts = 5;  //CTS Pin des COM-Ports

long count = 0;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT); 
 
  pinMode(cts, OUTPUT);
  digitalWrite(cts, LOW);
 
  p50x.setup(57600);   //Initialisierung P50x
  
  dcc.setup(6);   //DCC OUT PIN
}

// the loop routine runs over and over again forever:
void loop() {
  
  p50x.receive();
  
  dcc.update();
  count++;
  
  if (count > 10000) {
    p50x.xLokStsUpdate();  //Update Informationen
    count = 0;
  }
  
}

void notifyRS232 (uint8_t State)
{
  digitalWrite(led, State);
}

void notifyTrntRequest( uint16_t Address, uint8_t State, uint8_t Direction, uint8_t Lock )
{
  dcc.setBasicAccessoryPos(Address, (Direction & 0x01));
/*  
  Serial.print("Switch Request: ");
  Serial.print(Address, DEC);
  Serial.print(':');
  Serial.print(Direction ? "closed" : "thrown");
  Serial.print(" - ");
  Serial.print(State ? "ein" : "aus");
  Serial.print(" - Locked: ");
  Serial.println(Lock ? "On" : "Off");
*/  
}

void notifyLokRequest( uint16_t Address, uint8_t Speed, uint8_t Direction, uint8_t F0)
{
  int8_t spd = Speed;
  bitWrite(spd, 7, Direction); 
  dcc.setSpeed(Address, spd, 128);
  dcc.setFunctions0to4(Address,F0);
    
 /*
  Serial.print("XLok Request - Adr: ");
  Serial.print(Address, DEC);
  Serial.print(':');
  Serial.print(Direction ? "vorwaerts" : "rueckwaerts");
  Serial.print(" - Speed: ");
  Serial.print(Speed, DEC);
  Serial.print(" - lights=");
  Serial.println(F0 ? "on" : "off");
*/
}

void notifyLokF1F4Request( uint16_t Address, uint8_t Function )
{
  dcc.setFunctions0to4(Address, Function);

  
/*  
  Serial.print("Function Request - Adr: ");
  Serial.print(Address, DEC);
  Serial.print(", F4-F1: ");
  Serial.println(Function, BIN);
  */
}

void notifyLokFuncRequest( uint16_t Address, uint8_t Function )
{
  dcc.setFunctions0to4(Address,Function & 0x0F); //override F0 if active !!!
  dcc.setFunctions5to8(Address,(Function & 0xF0) >> 4);

/*
  Serial.print("Func Request - Adr: ");
  Serial.print(Address, DEC);
  Serial.print(", F8-F1: ");
  Serial.println(Function, BIN);
  */
}

void notifyLokFunc2Request( uint16_t Address, uint8_t Function2 )
{
  dcc.setFunctions9to12(Address,Function2);

/*  Serial.print("Func Request - Adr: ");
  Serial.print(Address, DEC);
  Serial.print(", F16-F9: ");
  Serial.println(Function2, BIN);
  */
}

void notifyLokFunc34Request( uint16_t Address, uint8_t Function3, uint8_t Function4 )
{
  dcc.setFunctions13to20(Address,Function3);
  dcc.setFunctions21to28(Address,Function4);
/*  
  Serial.print("Func Request - Adr: ");
  Serial.print(Address, DEC);
  Serial.print(", F24-F17: ");
  Serial.print(Function3, BIN);
  Serial.print(", F28-F25: ");
  Serial.println(Function4, BIN);*/
}
					
void notifyPowerRequest (uint8_t Power) {
  //Serial.println(Power ? "Power ON" : "Power OFF");
//  if (Power == false)
//    dcc.eStop();
  dcc.setpower(Power);  
}

//PoM Mode
void notifyXDCC_PDRRequest( uint16_t Address, uint16_t CVAddress ) {
/*  Serial.print("XDCC_PDR PoM Request - Adr: ");
  Serial.print(Address, DEC);
  Serial.print(", CV: ");
  Serial.println(CVAddress, DEC);*/
  //p50x.ReturnPT(byte CV-Wert);
}


void notifyXDCC_PDRequest( uint16_t Address, uint16_t CVAddress, uint8_t Value ) {
  dcc.opsProgramCV(Address, CVAddress, Value);
  
/*  
  Serial.print("XDCC_PDR PoM Request - Adr: ");
  Serial.print(Address, DEC);
  Serial.print(", CV: ");
  Serial.print(CVAddress, DEC);
  Serial.print(", Value: ");
  Serial.println(Value, DEC);
  */
}

//Paged Mode
void notifyXPT_DCCRPRequest( uint16_t CVAddress ) {	//lesen
/*  Serial.print("Paged Mode Request - CVAdr: ");
  Serial.println(CVAddress, DEC);*/
}
void notifyXPT_DCCWPRequest( uint16_t CVAddress, uint8_t Value ) {	//schreiben
/*  Serial.print("Paged Mode Request - CVAdr: ");
  Serial.print(CVAddress, DEC);
  Serial.print(", Value: ");
  Serial.println(Value, DEC);*/
}

//Bit-Mode
void notifyXPT_DCCRBRequest( uint16_t CVAddress ) {	//lesen
/*  Serial.print("Bit-Mode Request - CVAdr: ");
  Serial.println(CVAddress, DEC);*/
}
void notifyXPT_DCCWBRequest( uint16_t CVAddress, uint8_t Bit, uint8_t Value ) {	//schreiben
/*  Serial.print("Bit-Mode Request - CVAdr: ");
  Serial.print(CVAddress, DEC);
  Serial.print(", Value: ");
  Serial.println(Value, DEC);*/
}

//Direct-Mode
void notifyXPT_DCCRDRequest( uint16_t CVAddress ) {	//lesen
/*  Serial.print("Direct-Mode Request - CVAdr: ");
  Serial.println(CVAddress, DEC);*/
}
void notifyXPT_DCCWDRequest( uint16_t CVAddress, uint8_t Value ) {	//schreiben
/*  Serial.print("Direct-Mode Request - CVAdr: ");
  Serial.print(CVAddress, DEC);
  Serial.print(", Value: ");
  Serial.println(Value, DEC);*/
}

//Programmierung Rückmelden
byte notifyXPT_TermRequest() {
return 0xF3;	//no task is active
}

