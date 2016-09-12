//--------------------------------------------------------------
/*

  LocoNet Master Interface
  
Funktionsumfang:  
- Fahren aber nicht per Slot Write
- Dispatch (put via Z21)

Copyright (c) by Philipp Gahtow, year 2015
  
*/
#if defined(LOCONET)

//**************************************************************
//Setup up PIN-Configuration for different MCU
#include "MCU_config.h"

//config:
#if defined(UNO_MCU) || defined(__AVR_ATmega644P__)
#define MaxSlot 10
#else
#define MaxSlot 120    //max. 120 Slots
#endif

typedef struct	//SLOT
{
  uint8_t Adr;
  uint8_t Adr2;
  uint8_t Status;  
} TypeSlot;

TypeSlot slot[MaxSlot];

lnMsg        *LnPacket;
LnBuf        LnTxBuffer;

byte dispatchSlot = 0;    //To put and store a SLOT for DISPATCHING
boolean dpgetSLot = false;
//byte LNSendTX = 0;    //Z21 hat eine Meldung auf den LocoNet-Bus geschrieben?
boolean LNgetNext = false; //Z21 hat eine Meldung auf den LocoNet-Bus geschrieben?
boolean LNNextTX = false;
//--------------------------------------------------------------------------------------------
//Define new OPC:
#define OPC_UHLI_FUN   0xD4  //Function 9-28 by Uhlenbrock 
#define OPC_MULTI_SENSE 0xD0  //power management and transponding

#if defined(LnDEB)
//Print LN Message
void LNDebugPrint () {
  uint8_t msgLen = getLnMsgSize(LnPacket);   //get Length of Data
  // First print out the packet in HEX
        for (uint8_t x = 0; x < msgLen; x++)  {
          uint8_t val = LnPacket->data[x];
            // Print a leading 0 if less than 16 to make 2 HEX digits
          if(val < 16)
            Debug.print('0');
          Debug.print(val, HEX);
          Debug.print(' ');
        }
        Debug.println();
}
#endif

//--------------------------------------------------------------------------------------------
//send bytes into LocoNet
bool LNSendPacket (byte *data, byte length) 
{
  lnMsg SendPacket;
  for (byte i = 0; i < (length-1); i++) {
    SendPacket.data[i] = *data;
    data++;
  }
  LocoNet.send( &SendPacket);
  LnPacket = LocoNet.receive();
  if (SendPacket.data[length-1] == LnPacket->data[length-1]) {
    #if defined(LnDEB)
    Debug.print("STX: ");
    LNDebugPrint();
    #endif
  }
  else LNgetNext = true;
  return true;
/*  
  static lnMsg   *LoconetPacket;
  byte XOR = 0xFF;
  for (byte i = 0; i < (length-1); i++) {
    addByteLnBuf( &LnTxBuffer, *data);
    XOR = XOR ^ *data;
    data++;
  }
  addByteLnBuf( &LnTxBuffer, XOR ) ;    //Trennbit
  addByteLnBuf( &LnTxBuffer, 0xFF ) ;    //Trennbit
  // Check to see if we have received a complete packet yet
  LoconetPacket = recvLnMsg( &LnTxBuffer );    //Senden vorbereiten
  if(LoconetPacket ) {        //korrektheit Prüfen
    LocoNet.send( LoconetPacket );  // Send the received packet from the PC to the LocoNet
    LNSendTX++;  //Meldung auf LocoNet geschreiben (TX) -> notify z21 LAN also
    
    LnPacket = LocoNet.receive() ;
    #if defined(LnDEB)
    Debug.print("STX: ");
    LNDebugPrint();
    #endif
    
    return true;
  }  
  return false;
  */
}

//--------------------------------------------------------------------------------------------
void LNGetLocoStatus(byte Slot) {
  dcc.getLocoStateFull((slot[Slot].Adr2 << 7) | slot[Slot].Adr, false);
}

#if defined(LNMaster)
//--------------------------------------------------------------------------------------------
byte LNGetSetLocoSlot(byte Adr2, byte Adr, bool add) {
  if (Adr2 == 0 && Adr == 0)
    return 0;
  byte getSlot = 0;
  for (byte i = 1; i < MaxSlot; i++) {
    if (slot[i].Adr == Adr && slot[i].Adr2 == Adr2)
      return i;  //already inside a SLOT
    if (getSlot == 0 && slot[i].Status == 0)
      getSlot = i;  //find a empty SLOT  
  }
  if (getSlot != 0 && add == true) {    //add Adr to SLOT Server!
    slot[getSlot].Status = 0x10;   //B00000111;  //Slot Status inkl. Geschwindigkeitsstufen 
    slot[getSlot].Adr = Adr;
    slot[getSlot].Adr2 = Adr2;
    return getSlot;
  }
  return 0;
}
#else //LN Slave-Mode
//--------------------------------------------------------------------------------------------
//Find Slot via Address
byte LNGetSetLocoSlot (byte Adr2, byte Adr, bool add) {
  if (Adr == 0 && Adr2 == 0)
    return 0;
  for (byte i = 1; i < MaxSlot; i++) {
    if ((slot[i].Adr == Adr) && (slot[i].Adr2 == Adr2)) { // && (getLNSlotState(Slot) != 0)) {
      return i;    //Vorhanden!
    }
  }
  //If not: Master puts Address into a SLOT
  byte getSlot[] = {OPC_LOCO_ADR, Adr2, Adr, 0x00};  //Request loco address
  LNSendPacket(getSlot, 4);  
  return 0;
}
//--------------------------------------------------------------------------------------------
//Set slot direction, function 0-4 state 
// DIRF = 0,0,DIR,F0,F4,F3,F2,F1 
void sendLNDIRF (unsigned int Adr, byte DIRF) {
  byte Slot = LNGetSetLocoSlot(Adr >> 8, Adr & 0xFF, TXAllLokInfoOnLN);
  byte setDIRF[] = {OPC_LOCO_DIRF, Slot, DIRF, 0x00};
  LNSendPacket(setDIRF, 4);  
}
//--------------------------------------------------------------------------------------------
//Set slot speed
void sendLNSPD (unsigned int Adr, byte SPD) {
  byte Slot = LNGetSetLocoSlot(Adr >> 8, Adr & 0xFF, TXAllLokInfoOnLN);
  if (dcc.getLocoDir(Adr) != (SPD >> 7)) {
      byte DIRF = dcc.getFunktion0to4(Adr) | (dcc.getLocoDir(Adr) << 5);
      byte setDIRF[] = {OPC_LOCO_DIRF, Slot, DIRF, 0x00};
      LNSendPacket(setDIRF, 4);
  }
  byte setSPD[] = {OPC_LOCO_SPD, Slot, SPD & B1111111, 0x00};
  LNSendPacket(setSPD, 4);  
}
//--------------------------------------------------------------------------------------------
//Set slot second function
// SND = 0,0,0,0,F8,F7,F6,F5 
void sendLNSND (unsigned int Adr, byte SND) {
  byte Slot = LNGetSetLocoSlot(Adr >> 8, Adr & 0xFF, TXAllLokInfoOnLN);
  byte setSND[] = {OPC_LOCO_SND, Slot, SND, 0x00};
  LNSendPacket(setSND, 4);  
}
#endif  //LN Slave Mode

//--------------------------------------------------------------------------------------------
//Check if Slot can be dispatched
byte LNdispatch (byte Adr2, byte Adr) {
      Debug.print("Null Move ");
  dispatchSlot = LNGetSetLocoSlot(Adr2, Adr, true);  //add to SLOT
  #if !defined(LNMaster)  //At Slave Mode ask Master to dispatch 
   if (dispatchSlot > 0) {
    byte SetStat1[] = {OPC_SLOT_STAT1, dispatchSlot, 0x20, 0x00};
    LNSendPacket(SetStat1, 4); 
    byte NullMove[] = {OPC_MOVE_SLOTS, dispatchSlot, 0x00, 0x00};
    LNSendPacket(NullMove, 4); 
   }
   else { //no Slot for loco that should be dispatched - get a Slot!
      dpgetSLot = true;  //get ready for Slot
      Debug.print(" get Slot");
      return 0;
   }
  #endif
  if (((slot[dispatchSlot].Status >> 4) & 0x03) != B11) {  //not 11=IN_USE
    return dispatchSlot;
  }
  dispatchSlot = 0;  //clear
  return 0;
}

//--------------------------------------------------------------------------------------------
//LocoNet Interface init
void LNsetup() {
  // First initialize the LocoNet interface
  LocoNet.init(LNTxPin);
  initLnBuf(&LnTxBuffer);
}

//--------------------------------------------------------------------------------------------
//Send railpower state to LocoNet devices
void LNsetpower() {
  byte code[] = { OPC_GPOFF, 0x00};
  if (Railpower == csNormal)
    code[0] = OPC_GPON;
  else if (Railpower == csEmergencyStop)
    code[0] = OPC_IDLE;  //B'cast emerg. STOP
  LNSendPacket(code,2);
  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, true);   
}

//--------------------------------------------------------------------------------------------
//Trnt Daten senden
void LNsetTrnt(int Adr, boolean state, boolean active) {
  //OPC_SW_REQ
  //dcc.setBasicAccessoryPos(LnPacket->data[1] | ((LnPacket->data[2] & 0x0F) << 7),(LnPacket->data[2] >> 5) & 0x01, (LnPacket->data[2] >> 4) & 0x01);  //Adr, State, on/off
  byte Trnt[] = {OPC_SW_REQ, Adr & B01111111, (Adr >> 7) & 0x0F, 0x00};
  bitWrite(Trnt[2], 5, state);
  bitWrite(Trnt[2], 4, active);
  LNSendPacket (Trnt, 4);  
  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, true);   
}

//--------------------------------------------------------------------------------------------
//LocoNet update via each loop
void LNupdate() {
  // Check for any received LocoNet packets
  if (LNgetNext == false)
    LnPacket = LocoNet.receive() ;
  if( LnPacket ) {
    #if defined(LnDEB)
    if (LNNextTX == true)
      Debug.print("TX: ");
    else Debug.print("RX: ");  
      LNDebugPrint();
    #endif
    if (LNNextTX == true) {
      LNNextTX = false;
      return;
    }
    
    if (LNgetNext == true) {
      LNNextTX = true;
      LNgetNext = false;
    }
      
    //Broadcast-Flag:
    byte LnZ21bcType = Z21bcLocoNet_s;  //Z21bcLocoNet or Z21bcLocoNetLocos or Z21bcLocoNetSwitches
    switch (LnPacket->data[0]) {  //OPC = Operation-Code
        case OPC_SL_RD_DATA: {  //for LN Slove-Mode ONLY
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), LnZ21bcType, false);   
                  byte Slot = LnPacket->data[2];  //Slot#
                  slot[Slot].Status = LnPacket->data[3];
                  slot[Slot].Adr = LnPacket->data[4];
                  slot[Slot].Adr2 = LnPacket->data[9];
                  #if !defined(LNMaster)  //At Slave Mode ask Master to dispatch 
                   if (dpgetSLot == true) {  //dispatch this Slot!
                    Debug.println("Dispatch");
                    if (slot[Slot].Status != 0x20) {
                      byte SetStat1[] = {OPC_SLOT_STAT1, Slot, 0x20, 0x00};
                      LNSendPacket(SetStat1, 4); 
                    }
                    byte NullMove[] = {OPC_MOVE_SLOTS, Slot, 0x00, 0x00};
                    LNSendPacket(NullMove, 4); 
                    dpgetSLot = false;  //reset
                   }
                  #endif
                  break;
        }
        case OPC_LOCO_ADR: { //0xBF = Request loco address
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), LnZ21bcType, false);   
                  //add to a SLOT:
                  byte newSlot = LNGetSetLocoSlot(LnPacket->data[1], LnPacket->data[2], true); //ADR2:7 ms-bits = 0 bei kurzer Adr; ADR:7 ls-bit
                  if (dispatchSlot != 0 && LnPacket->data[1] == 0 && LnPacket->data[2] == 0)
                    newSlot = dispatchSlot;
                  if (newSlot == 0) {
                    //0xB4 = OPC_LONG_ACK No free slot available
                    byte Fail[] = {OPC_LONG_ACK, OPC_LOCO_ADR & B01111111, 0x00, 0x00};
                    LNSendPacket (Fail, 4);
                    z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, false);   
                    break;
                  }
                  LNGetLocoStatus(newSlot);
                  break;
        }
        case OPC_MOVE_SLOTS: { //0xBA = Move slot SRC to DST
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), LnZ21bcType, false);   
                  #if defined(LNMaster) 
                  if (LnPacket->data[1] == 0) {  //SRC = 0
                    //SLOT READ DATA of DISPATCH Slot
                    if (dispatchSlot != 0) {
                      slot[dispatchSlot].Status = 0x33;  //IN_USE
                      LNGetLocoStatus(dispatchSlot); //Give slot that was DISPATCHED
                      dispatchSlot = 0;  //reset the Dispatch SLOT
                      break;
                    }
                    
                  }
                  else if (LnPacket->data[1] == LnPacket->data[2]) {  //NULL move
                    //SRC=DEST is set to IN_USE , if legal move -> NULL move
                    slot[LnPacket->data[1]].Status = 0x30;  //B00011111;  //IN_USE
                    LNGetLocoStatus(LnPacket->data[1]); 
                    break;
                  }
                  else if (LnPacket->data[2] == 0) {  //DST = 0
                    //DISPATCH Put, mark SLOT as DISPATCH;
                    dispatchSlot = LnPacket->data[1];
                    //RETURN slot status <0xE7> of DESTINATION slot DEST if move legal
                    LNGetLocoStatus(dispatchSlot); 
                    break;
                  }
                  //RETURN Fail LACK code if illegal move <B4>,<3A>,<0>,<chk>
                  byte Fail[] = {OPC_LONG_ACK, OPC_MOVE_SLOTS & B01111111, 0x00, 0x00};
                  LNSendPacket (Fail, 4);
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), LnZ21bcType, true);
                  break;
                  #endif
        }
        case OPC_RQ_SL_DATA: //Request slot data/status block
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), LnZ21bcType, false);   
                  LNGetLocoStatus(LnPacket->data[1]);
                  break;
        case OPC_LINK_SLOTS: 
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), LnZ21bcType, false);   
                  break;  //Link slot ARG1 to slot ARG2
        case OPC_UNLINK_SLOTS: 
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), LnZ21bcType, false);   
                  break;  //Unlink slot ARG1 from slot ARG2
        case OPC_SLOT_STAT1: 
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), LnZ21bcType, false);   
                  slot[LnPacket->data[1]].Status = LnPacket->data[2];
                  break;  
        case OPC_WR_SL_DATA: {  //Write slot data
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), LnZ21bcType, false);   
                  slot[LnPacket->data[2]].Adr = LnPacket->data[4];    //ADR
                  slot[LnPacket->data[2]].Adr2 = LnPacket->data[9];   //ADR2
                  slot[LnPacket->data[2]].Status = LnPacket->data[3];  //Save new Status
                  unsigned int Adresse = (LnPacket->data[9] << 7) | LnPacket->data[4];  // -> Adr
                  dcc.setSpeed128(Adresse, ((LnPacket->data[6] << 2) & B10000000) | (LnPacket->data[5] & B10000000));  //DIRF & SPD
                  dcc.setFunctions0to4(Adresse, LnPacket->data[6] & B00011111);	//DIRF = - F0 F4 F3 F2 F1
                  dcc.setFunctions5to8(Adresse, LnPacket->data[10]);	//SND = - F8 F7 F6 F5
                  dcc.getLocoStateFull(Adresse, false);      //request for other devices
                  //Response:
                  //0=busy/aborted, 1=accepted(OPC_SL_RD_DATA), 0×40=accepted blind(OPC_SL_RD_DATA), 0x7F=not implemented 
                  byte ACK[] = {OPC_LONG_ACK, OPC_WR_SL_DATA & B01111111, 1, 0x00};  
                  LNSendPacket (ACK, 4);    //Send ACK
                  LNGetLocoStatus(LnPacket->data[2]);  //Send OPC_SL_RD_DATA
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), LnZ21bcType, true);   
                  break; 
        }
        case OPC_LOCO_SPD: {    //0SSSSSS
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetLocos_s, false);   
                  int Adresse = (slot[LnPacket->data[1]].Adr2 << 7) | slot[LnPacket->data[1]].Adr;
  //                if (LnPacket->data[2] == 0x7F)
  //                  LnPacket->data[2] -= 1;
                  dcc.setSpeed128(Adresse, (dcc.getLocoDir(Adresse) << 7) | LnPacket->data[2]);
                  dcc.getLocoStateFull(Adresse, false);      //request for other devices
                  break;    
        }  
        case OPC_LOCO_DIRF: { //0,0,DIR,F0,F4,F3,F2,F1
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetLocos_s, false);   
                  int Adresse = (slot[LnPacket->data[1]].Adr2 << 7) | slot[LnPacket->data[1]].Adr;
                  dcc.setSpeed128(Adresse, ((LnPacket->data[2] << 2) & B10000000) | dcc.getLocoSpeed(Adresse));
                  dcc.setFunctions0to4(Adresse, LnPacket->data[2] & B00011111);	//- F0 F4 F3 F2 F1
                  dcc.getLocoStateFull(Adresse, false);      //request for other devices
                  break;
        }
        case OPC_LOCO_SND: { //0,0,0,0,F8,F7,F6,F5 
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetLocos_s, false);   
                  int Adresse = (slot[LnPacket->data[1]].Adr2 << 7) | slot[LnPacket->data[1]].Adr;
                  dcc.setFunctions5to8(Adresse, LnPacket->data[2]);	//- F8 F7 F6 F5
                  dcc.getLocoStateFull(Adresse, false);      //request for other devices
                  break; 
        }
        case OPC_IMM_PACKET: { //Digitrax OPC_LOCO_F912 = Functions 9-12
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetLocos_s, false);   
                  byte ACK[] = {OPC_LONG_ACK, OPC_IMM_PACKET & B01111111, 0x00, 0x00};  //busy
                  LNSendPacket (ACK, 4);    //Send ACK 
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetLocos_s, true);            
                  break;
        }
        case OPC_UHLI_FUN: {  //Uhlenbrock OPC_LOCO_F912 = Function 9-28 by Uhlenbrock 
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetLocos_s, false);   
                  int Adresse = (slot[LnPacket->data[1]].Adr2 << 7) | slot[LnPacket->data[1]].Adr;
                  byte Func = 0x00;
                  if (LnPacket->data[2] == 0x07) { //Arg3
                    if ((LnPacket->data[3] & 0x10) != 0) 
                      Func |= B0001; //F9
                    if ((LnPacket->data[3] & 0x20) != 0) 
                      Func |= B0010; //F10
                    if ((LnPacket->data[3] & 0x40) != 0) 
                      Func |= B0100; //F11  
                    dcc.setFunctions9to12(Adresse, Func);	//- F12 F11 F10 F9
                  }
                  if (LnPacket->data[2] == 0x08)  //Arg3
                    dcc.setFunctions13to20(Adresse, LnPacket->data[3]);  //f13=0×01…f19=0×40
                  if (LnPacket->data[2] == 0x09)  //Arg3
                    dcc.setFunctions21to28(Adresse, LnPacket->data[3]);  //f21=0×01…f27=0×40
                  dcc.getLocoStateFull(Adresse, false);      //request for other devices
                  break;    
        }
        case OPC_SW_STATE: {  //Request state of switch. 
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetSwitches_s, false);   
                  //dcc.getBasicAccessoryInfo(Address+inc)
                  #if defined(LNMaster)
                  byte ACK[] = {OPC_LONG_ACK, LnPacket->data[0] & B01111111, 0x00, 0x00};  //Fail!!
                  LNSendPacket (ACK, 4);    //Send ACK
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetSwitches_s, true);   
                  #endif
                  break;
        }
        case OPC_SW_ACK: { //Request switch with acknoledge function.
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetSwitches_s, false);   
                  dcc.setBasicAccessoryPos(LnPacket->data[1] | ((LnPacket->data[2] & 0x0F) << 7),(LnPacket->data[2] >> 5) & 0x01, (LnPacket->data[2] >> 4) & 0x01);  //Adr, State, on/off
                  #if defined(LNMaster)
                  byte ACK[] = {OPC_LONG_ACK, LnPacket->data[0] & B01111111, 0x7F, 0x00};  //Succsess
                  LNSendPacket (ACK, 4);    //Send ACK
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetSwitches_s, true);   
                  #endif
                  break;
        }        
        case OPC_SW_REQ: //Request switch function 
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNetSwitches_s, false);   
                  dcc.setBasicAccessoryPos(LnPacket->data[1] | ((LnPacket->data[2] & 0x0F) << 7),(LnPacket->data[2] >> 5) & 0x01, (LnPacket->data[2] >> 4) & 0x01);  //Adr, State, on/off
                  break;
        case OPC_SW_REP: { //Turnout sensor state report 
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, false);   
                  //LnPacket->data[1] = 0,A6,A5,A4,A3,A2,A1,A0 
                  //LnPacket->data[2] =   0,X,I,L,A10,A9,A8,A7
                  byte Rdata[4];
                  Rdata[0] = 0x01; //Typ
                  Rdata[1] = LnPacket->data[2] & B1111;  //Adress A10-A8
                  Rdata[2] = (LnPacket->data[1] << 1) | ((LnPacket->data[2] >> 5) & 0x01);  //A7-A0
                  Rdata[3] = (LnPacket->data[2] >> 4) & 0x01; //L, Rückmelde-Zustand
                  z21.setLNDetector(Rdata, 4);
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, true);   
                  break;         
        }
        case OPC_INPUT_REP: { //0xB2 = Besetztmelder - LAN_LOCONET_DETECTOR
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, false);   
                  //LnPacket->data[1] = 0,A6,A5,A4,A3,A2,A1,A0 
                  //LnPacket->data[2] = 	0,X,I,L,A10,A9,A8,A7
                  byte Rdata[4];
                  Rdata[0] = 0x01; //Typ
                  Rdata[1] = LnPacket->data[2] & B1111;  //Adress A10-A8
                  Rdata[2] = (LnPacket->data[1] << 1) | ((LnPacket->data[2] >> 5) & 0x01);  //A7-A0
                  Rdata[3] = (LnPacket->data[2] >> 4) & 0x01; //L, Rückmelde-Zustand
                  z21.setLNDetector(Rdata, 4);
                  #if defined(REPORT)
                  Debug.print("LN Sensor:");
                  //Debug.print(((LnPacket->data[1] << 1) | ((LnPacket->data[2] & B1111) << 8) | ((LnPacket->data[2] >> 5) & 0x01)) + 1);
                  Debug.print(word(Rdata[1],Rdata[2])+1);
                  Debug.println(((LnPacket->data[2] >> 4) & 0x01) ? "=on" : "=off");
                  #endif
                  break; }
        case OPC_MULTI_SENSE: {
                  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, false);   
                  byte Rdata[4];
                  Rdata[0] = LnPacket->data[1]; //Type
                  Rdata[1] = LnPacket->data[3]; //Adr
                  Rdata[1] = LnPacket->data[4]; //Adr
                  Rdata[3] = LnPacket->data[2]; //zone and section
                  z21.setLNDetector(Rdata, 4);  
                  break; }
        //Zustand Gleisspannung
        case OPC_GPOFF: z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, false);   
                        if (Railpower != csTrackVoltageOff) setPower(csTrackVoltageOff); break; 
        case OPC_GPON:  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, false);   
                        if (Railpower != csNormal) setPower(csNormal); break;
        case OPC_IDLE:  z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, false);   
                        if (Railpower != csEmergencyStop) setPower(csEmergencyStop); break;
      }
   }
}

//--------------------------------------------------------------------------------------------
//Status change for a loco
void LNSetLocoStatus(unsigned int Adr, byte Speed, byte F0, byte F1) { //UserOps=SLOT,Speed,F0,F1
  byte Slot = LNGetSetLocoSlot(Adr >> 8, Adr & 0xFF, TXAllLokInfoOnLN);
  if (Slot != 0) {    //wenn Lok im SLOT Server vorhanden ist --> update
    // DIRF = 0,0,DIR,F0,F4,F3,F2,F1 
    // SND = 0,0,0,0,F8,F7,F6,F5 
    
    byte DIRF = ((Speed >> 2) & B00100000) | F0;
    byte SLOT_DATA_READ[] = {OPC_SL_RD_DATA, 0x0E, Slot, slot[Slot].Status, slot[Slot].Adr, Speed & B01111111, DIRF, 0, 0, slot[Slot].Adr2, F1 & 0x0F, 0, 0};
    LNSendPacket (SLOT_DATA_READ, 0x0E);
    z21.setLNMessage(LnPacket->data, getLnMsgSize(LnPacket), Z21bcLocoNet_s, true);   
  }
}


//--------------------
#endif
