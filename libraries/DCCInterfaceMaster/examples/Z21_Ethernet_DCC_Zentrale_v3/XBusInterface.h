//--------------------------------------------------------------
/*

  XpressNetMaster Interface for Arduino
  
Funktionsumfang:  
- Fahren per LokMaus2 und MultiMaus
- Schalten von DCC Weichen mit der MultiMaus (not tested 15.04.15)
  
*/
#if defined(XPRESSNET)

XpressNetMasterClass XpressNet;

//Pin Config:
#define XNetTxRxPin  9            //Port for Send/Receive on MAX485
#define XNetFahrstufen Loco128    //Standard Fahrstufen Setup

byte XNetUserOps = 0x00;
boolean XNetReturnLocoInfo = false;
boolean XNetReturnLocoFkt = false;

//--------------------------------------------------------------
//Change Power Status
void notifyXNetPower(uint8_t State) {
  #if defined(DEBUG)
  Serial.print("XNetPower: ");
  Serial.println(State, HEX);
  #endif
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
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", S14:");
  Serial.println(Speed, BIN);
  #endif
  if (Speed == 0) 
    dcc.setSpeed14(Address, (dcc.getLocoDir(Address) << 7) | (Speed & B01111111));
  else dcc.setSpeed14(Address, Speed);
}

//--------------------------------------------------------------
void notifyXNetLocoDrive28(uint16_t Address, uint8_t Speed) {
  #if defined(DEBUG)
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", S28:");
  Serial.println(Speed, BIN);
  #endif
  if (Speed == 0)
    dcc.setSpeed28(Address, (dcc.getLocoDir(Address) << 7) | (Speed & B01111111));
  else dcc.setSpeed28(Address, Speed);
}

//--------------------------------------------------------------
void notifyXNetLocoDrive128(uint16_t Address, uint8_t Speed) {
  #if defined(DEBUG)
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", S128:");
  Serial.println(Speed, BIN);
  #endif
  if (Speed == 0) 
    dcc.setSpeed128(Address, (dcc.getLocoDir(Address) << 7) | (Speed & B01111111));
  else dcc.setSpeed128(Address, Speed);
}

//--------------------------------------------------------------
void notifyXNetLocoFunc1(uint16_t Address, uint8_t Func1) {
  #if defined(DEBUG)
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", F1:");
  Serial.println(Func1, BIN);
  #endif
  dcc.setFunctions0to4(Address, Func1);	//- F0 F4 F3 F2 F1
}

//--------------------------------------------------------------
void notifyXNetLocoFunc2(uint16_t Address, uint8_t Func2) {
  #if defined(DEBUG)
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", F2:");
  Serial.println(Func2, BIN);
  #endif
  dcc.setFunctions5to8(Address, Func2);	//- F8 F7 F6 F5
}

//--------------------------------------------------------------
void notifyXNetLocoFunc3(uint16_t Address, uint8_t Func3) {
  #if defined(DEBUG)
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", F3:");
  Serial.println(Func3, BIN);
  #endif
  dcc.setFunctions9to12(Address, Func3);	//- F12 F11 F10 F9
}

//--------------------------------------------------------------
void notifyXNetTrntInfo(uint8_t UserOps, uint8_t Address, uint8_t data) {
  byte inc = 0;  //lower Nibble
  if (data == 0x01)
    inc = 2;  //upper Nibble
  byte pos = 0x00;
  if (dcc.getBasicAccessoryInfo(Address+inc) == true)
    bitWrite(pos, 0, 1);
  if (dcc.getBasicAccessoryInfo(Address+inc+1) == true)
    bitWrite(pos, 1, 1);  
  XpressNet.SetTrntStatus(UserOps, Address, pos);
}

//--------------------------------------------------------------
void notifyXNetTrnt(uint16_t Address, uint8_t data) {
  #if defined(DEBUG)
  Serial.print("XNet TA:");
  Serial.print(Address);
  Serial.print(", P:");
  Serial.println(data, BIN);
  #endif
  if (data == 0x01)
    dcc.setBasicAccessoryPos(Address,true);
  else dcc.setBasicAccessoryPos(Address,false);
}

#endif
