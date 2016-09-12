//--------------------------------------------------------------
/*

  DCC Decoder Interface

  - decode NMRA DCC Data for loco, function and accessory
  - react on change interrupt and take time between
  - store the received Data
  

  Copyright (c) by Philipp Gahtow, year 2015
  
*/
#if defined(DECODER)

//**************************************************************
//Setup up PIN-Configuration for different MCU
#include "MCU_config.h"

//config:
#define maxSaved 30        //numbers of Data that is saved to compare a new packet

//Variablen für DCC Erkennung
unsigned long DCCMicros = 0;    //Time to check DCC Data "1" or "0"

byte countone = 0;    //Zähle die high so das man eine Präambel erkennen kann (min. 10 high)
boolean getdata = false;     //Es werden DCC Daten gelesen und im Array data abgespeichert
boolean dataReady = false;   //Es wurden DCC Daten vollständig eingelesen
byte decDCCData[5];          //Übergabe eingelesene Byte-Werte nach einer Präambel
byte DCCData[5];          //eingelesene Byte-Werte nach einer Präambel
byte datalength = 0;  //Position in data wo gerade der nächste Wert hinkommt
byte countbit = 0;    //Zähle Bits, nach 8 Bit wurde ein Byte gelesen und setzte dann die Variable auf 0
byte dxor = 0;        //Übergabe der Prüfsumme
byte cxor = 0;         //xor Prüfsumme
unsigned int dccAdr = 0;          //Adresse

#define DCC_SHORT_ADDRESS           0x00
#define DCC_LONG_ADDRESS            0x01
#define DCC_ACC_ADDRESS             0x02

byte SavedXOR[maxSaved];        //The XOR of the last circles
uint16_t SavedADR[maxSaved];    //The Adress and kind of data send in the last circles
byte SavedCount = 0;

//****************************************************************   
//Intterrupt auf DCC-Pin
void readDCCData()
{
  unsigned long micro = micros();  
  if (digitalRead(decDCCPin) == HIGH) {
    DCCMicros = micro;    //Speichern der Zeit
    return;  
  }
  if (getdata == true) 
    countbit += 1;    //Abzählen der Bytes, also jeweils 8 Bit
  if (micro - DCCMicros < 80) {      //1-Bit gelesen; over 68us
    countone += 1;            //Zählen der 1-Bit für Präambel erforderlich
    if (getdata == true && countbit <= 8) {    //eingelesenen Bitwert im Array Speichern
      bitWrite(DCCData[datalength], 8-countbit, 1);    //Speichert das ein 1-Bit gelesen wurde
    }
    if (countbit > 8) {        //End-Bit gelesen.
      countbit = 0; 
      countone = 0;            //lösche Anzahl gelesener 1-Bit
      getdata = false;        //Stop des Einlesen der Daten
      //XOR Prüfen:
        //Prüfen von XOR und letztes Byte
      if (DCCData[datalength] != cxor) { // || dataReady == true)
        cxor = 0;          //XOR zurücksetzten
        datalength = 0;    //Position im Array an der die Bitwerte gespeichert werden.
        dataReady == false;
        return;    //verwerfen!
      }
      for (byte i = 0; i < 5; i++) {    //copy data
        if (i > datalength)
          decDCCData[i] = 0;
        else decDCCData[i] = DCCData[i];  
      }
      dxor = cxor;
      
      dataReady = true;  //Fertig, Daten Auswerten!
    }
  }  //Ende 1-Bit
  else {                  //0-Bit gelesen
    if (getdata == true && countbit <= 8) {    //eingelesenen Bitwert im Array Speichern
      bitWrite(DCCData[datalength], 8-countbit, 0);    //Speichert das ein 0-Bit gelesen wurde
    }
    if (countone > 10) {   //Präambel erkannt ab hier Daten lesen. (Es wurden mind. 10 HIGH erkannt)
      getdata = true;     //Einlesen der Bitwerte in Array aktivieren.
      datalength = 0;    //Position im Array an der die Bitwerte gespeichert werden.
      countbit = 0;       //beginne Bits zu zählen. Auswertung von 1 Byte Blöcken ermöglichen nach 8 Bit.
      cxor = 0;          //XOR zurücksetzten
    }
    if (countbit > 8) {    //Null-Bit gelesen. (Immer nach 1 Byte)
      countbit = 0;
      cxor = cxor ^ DCCData[datalength];  //XOR bestimmen!
      datalength += 1;  //Bitweise im Array weitergehen.
    }
    countone = 0;    //Es wurde ein 0-Bit gelesen, lösche Anzahl gelesener 1-Bit
  }  //Ende 0-Bit
}

//****************************************************************  
//Auswerten der eingelesenen Bitwerte.
void DCCDecoder_anaylse() {
  /*
    Beschreibung der Einträge im data Array:
      decDCCData[0]   : 1. Byte nach der Präambel (Adresse)
      Bit 8     : Null-Bit
      decDCCData[1]   : 2. Byte (long Adresse oder Daten)
      Bit 17    : Null-Bit
      decDCCData[2]   : 3. Byte (bei datalength == 26 => CRC)
      Bit 26    : Null-Bit (bei datalength == 26 => End-Bit)
      decDCCData[3]   : 4. Byte (wenn vorhanden, long Adr. oder long Data)
      Bit 35    : End-Bit (wenn vorhanden, long Adr. oder long Data)
      decDCCData[4]   : 5. Byte (wenn vorhanden, long Adr. und long Data)
      Bit 44    : End-Bit (wenn vorhanden, long Adr. und long Data)
  */
 
    //1.1 Bestimmen kurzen Adresse (bis 127)
  if (bitRead(decDCCData[0],7) == 0) {
    dccAdr = decDCCData[0];
    if (dccAdr == 0xFF || dccAdr == 0)  //idle
      return;
  }

  //1.2 Bestimmen der langen Adresse mit 14 Bits also 16384
  byte verschub = 0;
  if ((decDCCData[0] >> 6) == B11) {
    dccAdr = word(decDCCData[0] & B00111111, decDCCData[1]);
    verschub = 1; //langes Datenpacket
    if (dccAdr >= 12000) {  //idle
      return;
    }
  }
  
//--------------------------------------------------------------
  //1.3 Bestimmen der Weichenadresse
  if (((decDCCData[0] >> 6) == 0x02) && ((decDCCData[1] >> 7) == 0x01)) {  
    dccAdr = (((~decDCCData[1]) & 0x70) << 2) | (decDCCData[0] & 0x3F); //Adresse
    dccAdr = (dccAdr << 2) | ((decDCCData[1] >> 0x01) & 0x03);  //Port bestimmen, bis zu 4 möglich
    dcc.setBasicAccessoryPos(dccAdr, decDCCData[1] & 0x01, (decDCCData[1] >> 3) & 0x01); //Adr, left/right, activ
    
    #if defined(LOCONET)
    LNsetTrnt(dccAdr, decDCCData[1] & 0x01, bitRead(decDCCData[1],3));   //send to LocoNet
    #endif
    
    #if defined(DEBUG)
    Debug.print("EDCC W:");
    Debug.print(dccAdr);
    if (decDCCData[1] & B1 == 1)
      Debug.print(" gerade");
    else Debug.print(" Abzweig");
    if ((decDCCData[1] >> 3) & B1 == 1)
      Debug.print(" activ");
    Debug.println();
    #endif
    return;
  }
//--------------------------------------------------------------

  //1.3 Check if already worked on this packet:
  if (decDCCData[1+verschub] == B11011111)  //ignore F20 bis F28: Packet beginnt auch mit "110"!
   return;
  uint16_t thisADR = (decDCCData[1+verschub] >> 5) | (dccAdr << 3); 
  byte counter;
  for (counter = 0; counter < maxSaved; counter++) {
    if (SavedADR[counter] == thisADR) {
      if (SavedXOR[counter] == dxor)
        return; //nothing to do here - same packet already send! 
      SavedXOR[counter] = dxor;  //SAVE  
      break;  
    }
  }
  if (SavedADR[counter] != thisADR) {
    SavedADR[SavedCount] = thisADR;
    SavedXOR[SavedCount] = dxor;  //SAVE
    SavedCount++;
    if (SavedCount == maxSaved) {
      SavedCount = 0;
    }
  }
  
  //dccType != DCC_ACC_ADDRESS:
  #if defined(DEBUG)
/*  
  Debug.print(thisADR);
  Debug.print(",");
  Debug.print(decDCCData[datalength]);
  Debug.print(":");
  Debug.print(dxor);
  Debug.print("=");
  Debug.print(counter);
  Debug.print("-");
*/  
  Debug.print(dcc.getLocoDir(dccAdr), BIN);
  Debug.print("EDCC A:");
  Debug.print(dccAdr);
  #endif
      if ((decDCCData[1+verschub] >> 5) == B100) {  //Funktionen F0 bis F4
        
        dcc.setFunctions0to4(dccAdr, decDCCData[1+verschub] & 0x1F);
        #if defined(LOCONET) && !defined(LnSLOTSRV)
        byte DIRF = (decDCCData[1+verschub] & 0x1F) | ((!dcc.getLocoDir(dccAdr)) << 5); //- F0 F4 F3 F2 F1
        sendLNDIRF(dccAdr, DIRF);
        #endif
        #if defined(DEBUG)
        Debug.print(DIRF, BIN);
        if (bitRead(decDCCData[1+verschub],4) == 1) 
            Debug.print(" F0");
        for (int i = 0; i < 4; i++) {
          if (bitRead(decDCCData[1+verschub],i) == 1) {
            Debug.print(" F");
            Debug.print(1+i);          
          }
        }
        if ((decDCCData[1+verschub] & B11111) == 0)
          Debug.print(" 0F off"); 
        Debug.println();
        #endif  
        return;
      }
      if ((decDCCData[1+verschub] >> 4) == B1011) {  //Funktionen F5 bis F8
        dcc.setFunctions5to8(dccAdr, decDCCData[1+verschub] & 0x0F);
        #if defined(LOCONET) && !defined(LnSLOTSRV)
        sendLNSND(dccAdr, decDCCData[1+verschub] & 0x0F);  //- F8 F7 F6 F5
        #endif
        #if defined(DEBUG)
        if (decDCCData[1+verschub] & 0x0F == 0)
          Debug.print(" F5to8 off");
        else {  
          for (int i = 0; i < 4; i++) {
            if (bitRead(decDCCData[1+verschub],i) == 1) {
              Debug.print(" F");
              Debug.print(5+i);          
            }
          }
        }
        Debug.println();
        #endif   
        return;
      }
      if ((decDCCData[1+verschub] >> 4) == B1010) {  //Funktionen F9 bis F12
        dcc.setFunctions9to12(dccAdr, decDCCData[1+verschub] & 0x0F);
        #if defined(DEBUG)
        if (decDCCData[1+verschub] & 0x0F == 0)
          Debug.print(" F9to12 off");
        else {  
          for (int i = 0; i < 4; i++) {
            if (bitRead(decDCCData[1+verschub],i) == 1) {
              Debug.print(" F");
              Debug.print(9+i);          
            }
          }
        }  
        Debug.println();
        #endif
        return;
      }
      if ((decDCCData[1+verschub] >> 6) == B01) {  //Fahrgeschwindigkeit/Fahrtrichtung 14 oder 28 Stufen
        uint8_t Vspeed = (decDCCData[1+verschub] & 0x0F) << 1; 
        bitWrite(Vspeed,0,bitRead(decDCCData[1+verschub],4));  //additional speed bit, move to bit 0!
        if (Vspeed > 3) {
          Vspeed = Vspeed - 3;    //4-31 to 1-28
          Vspeed = map(Vspeed, 1, 28, 2, 127);    //2-127
        }
        else {
          if (Vspeed > 1)  // 000010 E-Stop* 00011 E-Stop* (I)
            Vspeed = 1; 
          else Vspeed = 0;  
        }
        bitWrite(Vspeed, 7, bitRead(decDCCData[1+verschub],5));  //set DIR
        #if defined(DEBUG)
        if (bitRead(decDCCData[1+verschub],5) == 0)
          Debug.print(" #R:");
        else Debug.print(" #F:"); 
        #endif

        #if defined(DEBUG)
        switch (Vspeed & 0x7F) {
          case 0: Debug.print("STOP"); break;
          case 1: Debug.print("ESTOP"); break;
          default: Debug.print(Vspeed & 0x7F);
        }
        Debug.println();
        #endif

        #if defined(LOCONET) && !defined(LnSLOTSRV)
        sendLNSPD(dccAdr, Vspeed);
        #endif
          
        dcc.setSpeed128(dccAdr, Vspeed);
        return;
      }
      if (decDCCData[1+verschub] == B00111111) {  //126 Fahrstufen Auswerten
        #if defined(DEBUG)
        if (decDCCData[2+verschub] >> 7 == 0)
          Debug.print(" R:");
        else Debug.print(" F:");
        #endif
        uint8_t Vspeed = decDCCData[2+verschub];
               /*
        
        if (Vspeed > 1 && Vspeed < 128) //2-127
          Vspeed--;
        if (Vspeed > 129)
          Vspeed--; 
        */

        #if defined(DEBUG)
        switch (Vspeed & 0x7F) {
          case 0: Debug.print("STOP"); break;
          case 1: Debug.print("ESTOP"); break;
          default: Debug.print((Vspeed & 0x7F));
        }
        Debug.println();
        #endif
        
        #if defined(LOCONET) && !defined(LnSLOTSRV)
        sendLNSPD(dccAdr, Vspeed);
        #endif
           
        dcc.setSpeed128(dccAdr, Vspeed);
        return;
      }
      if (decDCCData[1+verschub] == B11011110) {  //Funktion F13 bis F20 im Expansion Byte lesen 
        dcc.setFunctions13to20(dccAdr, decDCCData[2+verschub]);
        #if defined(DEBUG)
        if (decDCCData[2+verschub] == 0)
          Debug.print(" F13to20 off");
        else {  
          for (int i = 0; i < 8; i++) {
            if (bitRead(decDCCData[2+verschub],i) == 1) {
              Debug.print(" F");
              Debug.print(13+i);          
            }
          }
        }
        Debug.println();
        #endif
      }
/*      
      if (decDCCData[1+verschub] == B11011111) {  //Funktion F21 bis F28 im Expansion Byte lesen 
        dcc.setFunctions21to28(dccAdr, decDCCData[2+verschub]);
        #if defined(DEBUG)
        if (decDCCData[2+verschub] == 0)
          Debug.print(" F21to28 off");
        else {  
          for (int i = 0; i < 8; i++) {
            if (bitRead(decDCCData[2+verschub],i) == 1) {
              Debug.print(" F");
              Debug.print(21+i);          
            }
          }
        }  
        Debug.println();
        #endif
      }  //ENDE F21 bis F28
*/      
//    dcc.getLocoStateFull(dccAdr, false);      //request for other devices
}

//****************************************************************  
//Call this in Setup routine
void DCCDecoder_init() {
  pinMode(decDCCPin, INPUT);    //Dateneingang
  attachInterrupt(0, readDCCData, CHANGE);  //ISR für den Dateneingang
}

//****************************************************************  
//Call this in each Loop round
void DCCDecoder_update() {
  if (dataReady == true) {    //Daten vom DCC Decoder   
    dataReady = false;
   DCCDecoder_anaylse();         //eingelesene Daten auswerten.
  }
}

#endif
