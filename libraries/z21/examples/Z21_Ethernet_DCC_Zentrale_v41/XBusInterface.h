//--------------------------------------------------------------
/*

  XpressNetMaster Interface for Arduino
  
Funktionsumfang:  
- Fahren per LokMaus2 und MultiMaus
- Schalten von DCC Weichen mit der MultiMaus (not tested 15.04.15)
  
*/
#if defined(XPRESSNET)

//**************************************************************
//Setup up PIN-Configuration for different MCU
#include "MCU_config.h"

XpressNetMasterClass XpressNet;

//config:
#define XNetFahrstufen Loco128    //Standard Fahrstufen Setup

byte XNetUserOps = 0x00;
boolean XNetReturnLocoInfo = false;
boolean XNetReturnLocoFkt = false;

//--------------------------------------------------------------
//Change Power Status
void notifyXNetPower(uint8_t State) {
  #if defined(DEBUG)
  Debug.print("XNetPower: ");
  Debug.println(State, HEX);
  #endif
  if (Railpower != State)
    setPower(State);
}

//--------------------------------------------------------------
void notifyXNetgiveLocoInfo(uint8_t UserOps, uint16_t Address) {
  XNetReturnLocoInfo = true;
  XNetUserOps = UserOps;
  dcc.getLocoStateFull(Address, false);
}

//--------------------------------------------------------------
void notifyXNetgiveLocoFunc(uint8_t UserOps, uint16_t Address) {
  XNetReturnLocoFkt = true;
  XNetUserOps = UserOps;
  dcc.getLocoStateFull(Address, false);
}

//--------------------------------------------------------------
void notifyXNetLocoDrive14(uint16_t Address, uint8_t Speed) {
  #if defined(DEBUG)
  Debug.print("XNet A:");
  Debug.print(Address);
  Debug.print(", S14:");
  Debug.println(Speed, BIN);
  #endif
  if (Speed == 0) 
    dcc.setSpeed14(Address, (dcc.getLocoDir(Address) << 7) | (Speed & B01111111));
  else dcc.setSpeed14(Address, Speed);
  dcc.getLocoStateFull(Address, false);      //request for other devices
}

//--------------------------------------------------------------
void notifyXNetLocoDrive28(uint16_t Address, uint8_t Speed) {
  #if defined(DEBUG)
  Debug.print("XNet A:");
  Debug.print(Address);
  Debug.print(", S28:");
  Debug.println(Speed, BIN);
  #endif
  if (Speed == 0)
    dcc.setSpeed28(Address, (dcc.getLocoDir(Address) << 7) | (Speed & B01111111));
  else dcc.setSpeed28(Address, Speed);
  dcc.getLocoStateFull(Address, false);      //request for other devices
}

//--------------------------------------------------------------
void notifyXNetLocoDrive128(uint16_t Address, uint8_t Speed) {
  #if defined(DEBUG)
  Debug.print("XNet A:");
  Debug.print(Address);
  Debug.print(", S128:");
  Debug.println(Speed, BIN);
  #endif
  if (Speed == 0) 
    dcc.setSpeed128(Address, (dcc.getLocoDir(Address) << 7) | (Speed & B01111111));
  else dcc.setSpeed128(Address, Speed);
  dcc.getLocoStateFull(Address, false);      //request for other devices
}

//--------------------------------------------------------------
void notifyXNetLocoFunc1(uint16_t Address, uint8_t Func1) {
  #if defined(DEBUG)
  Debug.print("XNet A:");
  Debug.print(Address);
  Debug.print(", F1:");
  Debug.println(Func1, BIN);
  #endif
  dcc.setFunctions0to4(Address, Func1);	//- F0 F4 F3 F2 F1
  dcc.getLocoStateFull(Address, false);      //request for other devices
}

//--------------------------------------------------------------
void notifyXNetLocoFunc2(uint16_t Address, uint8_t Func2) {
  #if defined(DEBUG)
  Debug.print("XNet A:");
  Debug.print(Address);
  Debug.print(", F2:");
  Debug.println(Func2, BIN);
  #endif
  dcc.setFunctions5to8(Address, Func2);	//- F8 F7 F6 F5
  dcc.getLocoStateFull(Address, false);      //request for other devices
}

//--------------------------------------------------------------
void notifyXNetLocoFunc3(uint16_t Address, uint8_t Func3) {
  #if defined(DEBUG)
  Debug.print("XNet A:");
  Debug.print(Address);
  Debug.print(", F3:");
  Debug.println(Func3, BIN);
  #endif
  dcc.setFunctions9to12(Address, Func3);	//- F12 F11 F10 F9
  dcc.getLocoStateFull(Address, false);      //request for other devices
}

//--------------------------------------------------------------
void notifyXNetLocoFunc4(uint16_t Address, uint8_t Func4) {
  #if defined(DEBUG)
  Debug.print("XNet A:");
  Debug.print(Address);
  Debug.print(", F4:");
  Debug.println(Func4, BIN);
  #endif
  dcc.setFunctions13to20(Address, Func4);	//F20 F19 F18 F17 F16 F15 F14 F13
  dcc.getLocoStateFull(Address, false);      //request for other devices
}

//--------------------------------------------------------------
void notifyXNetLocoFunc5(uint16_t Address, uint8_t Func5) {
  #if defined(DEBUG)
  Debug.print("XNet A:");
  Debug.print(Address);
  Debug.print(", F5:");
  Debug.println(Func5, BIN);
  #endif
  dcc.setFunctions21to28(Address, Func5);	//F28 F27 F26 F25 F24 F23 F22 F21
  dcc.getLocoStateFull(Address, false);      //request for other devices
}

//--------------------------------------------------------------
void notifyXNetTrntInfo(uint8_t UserOps, uint8_t Address, uint8_t data) {
  int adr = ((Address * 4) + ((data & 0x01) * 2));
  byte pos = data << 4;
  bitWrite(pos, 7, 1);  //command completed!
  if (dcc.getBasicAccessoryInfo(adr) == false)
    bitWrite(pos, 0, 1);
  else bitWrite(pos, 1, 1);  
  if (dcc.getBasicAccessoryInfo(adr+1) == false)
    bitWrite(pos, 2, 1);  
  else bitWrite(pos, 3, 1);    
  XpressNet.SetTrntStatus(UserOps, Address, pos);
}

//--------------------------------------------------------------
void notifyXNetTrnt(uint16_t Address, uint8_t data) {
    #if defined(DEBUG)
    Debug.print("XNet TA:");
    Debug.print(Address);
    Debug.print(", P:");
    Debug.println(data, BIN);
    #endif
    dcc.setBasicAccessoryPos(Address,data & 0x01, bitRead(data,3));    //Adr, left/right, activ
}

//--------------------------------------------------------------
void notifyXNetDirectCV(uint8_t CV, uint8_t data) {
  dcc.opsProgDirectCV(CV,data); 
}

#endif
