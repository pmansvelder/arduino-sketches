/********************
This is a simple command station that receive the commands via ethernet.
It base on the protocol from the Z21 of Roco!

- DCC Signal with Timer 1 by modifired CmdrArduino library
- fast S88 Bus use Timer 2
- DCC in to read signal from other Central Station via Interrupt 0 and Timer 4
- LocoNet only at MEGA with Timer 5 with Loconet.h library
- XpressNet via LOOP-Function with XpressNetMaster.h library
- HTTP Website on Port 80 to Configure
- Kehrschleifen output for ROCO Booster 

by Philipp Gahtow, year 2015

Change log Version 3.01:
15.04.2015  Abschaltung S88 Interface per Define (S88N)
16.04.2015  Aktualisierung Z21 LAN Protokoll V1.05 & Firmware-Version 1.26
17.04.2015  LN OPC_INPUT_REP msg von Belegmeldern über LAN_LOCONET_DETECTOR
20.04.2015  kurze/Lange DCC Adressen (1..99 kurz, ab 100 lang)
22.04.2015  Add in DCC Lib Function support F13 - F28
            Add Power Button with Reset (press at startup)
23.04.2015  Add LocoNet set Railpower (OPC_GPOFF, OPC_GPON, OPC_IDLE)
            Add LocoNet Slot Write (OPC_WR_SL_DATA)
            New Broadcast Msg (8 Bit) Z21 Protokoll V1.05 (Include LocoNet)
            Add LocoNet OPC_RQ_SL_DATA, OPC_UHLI_FUN, OPC_SW_REQ, OPC_SW_REP, OPC_SW_ACK, OPC_SW_STATE
28.04.2015  Add CV Write und Decoder Reset Packet before DCC CV-Programming            
----
toDo:
-> Rückmelder via XpressNet?
-> Programmieren von CVs (DCC & LocoNet)
-> store loco adr use via ip (max 16?)
********************/
/*---------------------------------------------------------------
Functions of the Command Station
=> uncomment if you not need them
*/
//**************************************************************
//Website to configure IP Address and Number of S88 Bus Module
#define HTTPCONF

//**************************************************************
//Singel S88 Bus Interface (max 62 * 8 Module)
#define S88N

//**************************************************************
//DCC Decoder (only for MEGA, To decode a DCC-Signal, add this data to command station DCC output)
#define DECODER   

//**************************************************************
//XpressNet Master Interface
#define XPRESSNET  
#include <XpressNetMaster.h>

//**************************************************************
//LocoNet Master Interface, only for MEGA!
#define LOCONET  
#include <LocoNet.h>

//**************************************************************
#define DEBUG      //To see DATA on Serial
//#define LnDEB    //To see HEX DATA of LocoNet Protokoll

//**************************************************************
//---------------------------------------------------------------
//DCC Master to create a DCC Signal:
//#include <DCCPacket.h>
//#include <DCCPacketQueue.h>
#include <DCCPacketScheduler.h>

#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library

#include <EEPROM.h>

#define EES88Moduls 38  //Adresse EEPROM Anzahl der Module für S88
#define EEip 40    //Startddress im EEPROM für die IP

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

//---------------------------------------------------------------
// The IP address will be dependent on your local network:
byte mac[] = { 0xFE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 188, 111);

//---------------------------------------------------------------
#if defined(HTTPCONF)
EthernetServer server(80);  // (port 80 is default for HTTP):
#endif

#define localPort 21105  //local port to listen Z21-Protokoll on UDP

//--------------------------------------------------------------
//Z21 Protokoll Typ Spezifikationen
#include "Z21type.h"

//--------------------------------------------------------------
//S88 Singel Bus:
#if defined(S88N)
#include "S88.h"
#endif

//--------------------------------------------------------------
//DCC Master:
#define DCCLed 3    //LED to show DCC active
#define DCCPin 6    //Pin for DCC sginal out
#define ShortPin 5  //Pin to detect Short Circuit of Booster
#define GOPin  A4   //Pin for GO/STOP Signal of Booster
#define KSPin  A5   //Pin for using Kehrschleifen-Modul
#define DetectShortCircuit 0xFF    //to detect short circuit
#define KSRelaisShortCircuit 0x1F  //to detect KS short
byte ShortTime = 0;            //Time Count for Short Detect
unsigned int LEDcount = 0;    //Timer for Status LED

DCCPacketScheduler dcc;

//--------------------------------------------------------------
// certain global XPressnet status indicators
#define csNormal 0x00 // Normal Operation Resumed ist eingeschaltet
#define csEmergencyStop 0x01 // Der Nothalt ist eingeschaltet
#define csTrackVoltageOff 0x02 // Die Gleisspannung ist abgeschaltet
#define csShortCircuit 0x04 // Kurzschluss
#define csServiceMode 0x08 // Der Programmiermodus ist aktiv - Service Mode
byte Railpower = csTrackVoltageOff;
bool Z21ButtonLastState = false;

//--------------------------------------------------------------
//Z21 Ethernet:
// buffers for receiving and sending data
#define UDP_TX_MAX_SIZE 12  //--> POM DATA has 12 Byte!
unsigned char packetBuffer[UDP_TX_MAX_SIZE]; //buffer to hold incoming packet,
//--> UDP_TX_PACKET_MAX_SIZE

struct TypeActIP {
  byte ip0;    // Byte IP
  byte ip1;    // Byte IP
  byte ip2;    // Byte IP
  byte ip3;    // Byte IP
  byte BCFlag;  //BoadCastFlag - see Z21type.h
  byte time;  //Zeit
};
TypeActIP ActIP[maxIP];    //Speicherarray für IPs

unsigned long IPpreviousMillis = 0;       // will store last time of IP decount updated
unsigned long ShortpreviousMillis = 0;    // will store last time of Short detect

//--------------------------------------------------------------
//DCC Decoder:
#if defined(DECODER)
#include "DCCDecoder.h"
#endif

//--------------------------------------------------------------
//XpressNet-Bus:
#if defined(XPRESSNET)
#include "XBusInterface.h"
#endif

//--------------------------------------------------------------
//LocoNet-Bus:
#if defined (LOCONET)
#include "LNInterface.h"
#endif

//--------------------------------------------------------------
void setup() {
  dcc.setup(DCCPin);
  
  pinMode(DCCLed, OUTPUT);      //DCC Status LED
  digitalWrite(DCCLed, LOW);    //DCC is in "off" State
  pinMode(KSPin, OUTPUT);       //Kehrschleife-Relais
  digitalWrite(KSPin, LOW);     //Kehrschleife
  pinMode(ShortPin, INPUT);	//set short pin
  digitalWrite(ShortPin, HIGH);  //Pull-UP
  pinMode(GOPin, OUTPUT);      //GO/STOP Signal
  digitalWrite(GOPin, LOW);    //set STOP to Booster
  
  pinMode(Z21ResetPin, INPUT_PULLUP);
  pinMode(Z21ButtonPin, INPUT_PULLUP);
  
  delay(10);
  
  #if defined(DEBUG)
    Serial.begin(115200);
    Serial.println("Z21");
  #endif
  
  if ((digitalRead(Z21ResetPin) == LOW) || (EEPROM.read(EEip) == 255)) {
      #if defined(DEBUG)
        Serial.println("RESET"); 
      #endif  
      EEPROM.write(EEip, ip[0]);
      EEPROM.write(EEip+1, ip[1]);
      EEPROM.write(EEip+2, ip[2]);
      EEPROM.write(EEip+3, ip[3]);
  }
  ip[0] = EEPROM.read(EEip);
  ip[1] = EEPROM.read(EEip+1);
  ip[2] = EEPROM.read(EEip+2);
  ip[3] = EEPROM.read(EEip+3);
  
  #if defined(S88N)
    SetupS88();    //Timer2 und S88 Setup  
  #endif  

  #if defined(DEBUG)
    Serial.println(ip);
    Serial.print("S88: ");
    #if defined(S88N)
      Serial.println(S88Module);
    #endif
    Serial.print("RAM: ");
    Serial.println(freeRam());  
  #endif  

  // start the Ethernet and UDP:
  Ethernet.begin(mac,ip);  //IP and MAC Festlegung
  Udp.begin(localPort);  //UDP Z21 Port
  
  #if defined(HTTPCONF)
    server.begin();    //HTTP Server
  #endif
  
  for (int i = 0; i < maxIP; i++)
    clearIPSlot(i);  //löschen gespeicherter aktiver IP's
    
  #if defined(XPRESSNET)  
    XpressNet.setup(XNetFahrstufen, XNetTxRxPin);    //Initialisierung XNet Serial und Send/Receive-PIN  
  #endif

  #if defined(DECODER)
    DCCDecoder_init();    //DCC Decoder init
  #endif
  
  #if defined(LOCONET)
    LNsetup();      //LocoNet Interface init
  #endif
}

//--------------------------------------------------------------------------------------------
void loop() {
  
  Ethreceive();    //Read Data on UDP Port
  
  updateDCCLed();     //DCC Status LED
  
  ShortDetection();  //handel short on rail => power off
  dcc.update();
  
  #if defined(HTTPCONF)
    Webconfig();    //Webserver for Configuration
  #endif
  
  #if defined(S88N)
    notifyS88Data();    //R-Bus geänderte Daten Melden
  #endif  
  
  #if defined(DECODER)
    DCCDecoder_update();    //Daten vom DCC Decoder
  #endif
  
  #if defined(XPRESSNET)  
    XpressNet.update();
  #endif
  
  #if defined(LOCONET)
    LNupdate();      //LocoNet update
  #endif
  
  //Button to control Railpower state
  if ((digitalRead(Z21ButtonPin) == LOW) && (Z21ButtonLastState == false)) {
    Z21ButtonLastState = true;
    switch(Railpower) {
      case csTrackVoltageOff: setPower(csNormal); break;
      case csEmergencyStop: setPower(csTrackVoltageOff); break;
      case csNormal: setPower(csEmergencyStop); break;
      default: setPower(csTrackVoltageOff); break;
    }
  }
  else {
    if ((digitalRead(Z21ButtonPin) == HIGH) && (Z21ButtonLastState == true))
      Z21ButtonLastState = false;
  }

  //Nicht genutzte IP's aus Speicher löschen
  unsigned long currentMillis = millis();
  if(currentMillis - IPpreviousMillis > interval) {
    IPpreviousMillis = currentMillis;   
    for (int i = 0; i < maxIP; i++) {
      if (ActIP[i].ip3 != 0) {  //Slot nicht leer?
        if (ActIP[i].time > 0) 
          ActIP[i].time--;    //Zeit herrunterrechnen
        else {
          #if defined(DEBUG)
            Serial.print("Clear ");
            Serial.println(ActIP[i].ip3);
          #endif  
          clearIPSlot(i);   //clear IP DATA
        }
      }
    }
  }
}

//--------------------------------------------------------------------------------------------
//POWER Configuration:
void setPower (byte state) {
  byte data[] = { LAN_X_BC_TRACK_POWER, 0x00  };
  Railpower = state;
  switch (state) {
    case csNormal: 
      dcc.setpower(ON);
      digitalWrite(GOPin, HIGH);
      data[1] = 0x01;
    break;
    case csTrackVoltageOff: 
      dcc.setpower(OFF);
      digitalWrite(GOPin, LOW);
      data[1] = 0x00;
    break;
    case csServiceMode: 
      dcc.setpower(ON);
      digitalWrite(GOPin, HIGH);
      data[1] = 0x02;
    break;
    case csShortCircuit: 
      data[1] = 0x08;
      dcc.setpower(ON);  //shut down via GO/STOP just for the Roco Booster
      digitalWrite(GOPin, LOW);
    break;
    case csEmergencyStop:
      dcc.eStop();  
      data[0] = LAN_X_BC_STOPPED; //0x81
      data[1] = 0x00;    
    break;
  }
  EthSend(0x07, 0x40, data, true, Z21bcAll_s);
  #if defined(XPRESSNET)
  XpressNet.setPower(Railpower);
  #endif
}

//--------------------------------------------------------------------------------------------
void ShortDetection() { 
  if (digitalRead(ShortPin) == LOW && Railpower != csShortCircuit) {  //Short Circuit!
    ShortTime++;
    if(ShortTime == DetectShortCircuit) {
        setPower(csTrackVoltageOff);
        setPower(csShortCircuit);
        #if defined(DEBUG)
        Serial.println("TRACK_SHORT_CIRCUIT");
        #endif
    }
    //Before Railpower cut out test change polarity:
    if (ShortTime == KSRelaisShortCircuit) {   
      digitalWrite(KSPin, !digitalRead(KSPin));     //Kehrschleife
      #if defined(DEBUG)
      Serial.print("KS ");
      Serial.println( digitalRead(KSPin) );
      #endif
    }
  }
  else ShortTime = 0;
}

//--------------------------------------------------------------------------------------------
void updateDCCLed () {
  if (LEDcount == 0) {  //flash, slow
    LEDcount = 0xF000;  //slow
    if (Railpower == csShortCircuit)
       LEDcount = 0xFEF0;    //fast
    if (Railpower == csNormal)
      digitalWrite(DCCLed, HIGH);
    else digitalWrite(DCCLed, !digitalRead(DCCLed));
  }
  LEDcount++;
}
/*
//--------------------------------------------------------------------------------------------
void clearIPSlots() {
  for (byte i = 0; i < maxIP; i++)
    clearIPSlot(i);
}
*/
//--------------------------------------------------------------------------------------------
//Slot mit Nummer "i" löschen
void clearIPSlot(byte i) {
  ActIP[i].ip0 = 0;
  ActIP[i].ip1 = 0;
  ActIP[i].ip2 = 0;
  ActIP[i].ip3 = 0;
  ActIP[i].BCFlag = 0;
  ActIP[i].time = 0;
}

//--------------------------------------------------------------------------------------------
void clearIPSlot(byte ip0, byte ip1, byte ip2, byte ip3) {
  for (int i = 0; i < maxIP; i++) {
    if (ActIP[i].ip0 == ip0 && ActIP[i].ip1 == ip1 && ActIP[i].ip2 == ip2 && ActIP[i].ip3 == ip3) 
      clearIPSlot(i);
  }
}

//--------------------------------------------------------------------------------------------
byte addIPToSlot (byte ip0, byte ip1, byte ip2, byte ip3, byte BCFlag) {
  byte Slot = maxIP;
  for (byte i = 0; i < maxIP; i++) {
    if (ActIP[i].ip0 == ip0 && ActIP[i].ip1 == ip1 && ActIP[i].ip2 == ip2 && ActIP[i].ip3 == ip3) {
      ActIP[i].time = ActTimeIP;
      if (BCFlag != 0)    //Falls BC Flag übertragen wurde diesen hinzufügen!
        ActIP[i].BCFlag = BCFlag;
      return ActIP[i].BCFlag;    //BC Flag 4. Byte Rückmelden
    }
    else if (ActIP[i].time == 0 && Slot == maxIP)
      Slot = i;
  }
  ActIP[Slot].ip0 = ip0;
  ActIP[Slot].ip1 = ip1;
  ActIP[Slot].ip2 = ip2;
  ActIP[Slot].ip3 = ip3;
  ActIP[Slot].time = ActTimeIP;
  setPower(Railpower);
  return ActIP[Slot].BCFlag;   //BC Flag 4. Byte Rückmelden
}

//--------------------------------------------------------------------------------------------
#if defined(HTTPCONF)
void Webconfig() {
  EthernetClient client = server.available();
  if (client) {
    String receivedText = String(50);
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (receivedText.length() < 50) {
          receivedText += c;
        }
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          //client.println("Connection: close");  // the connection will be closed after completion of the response
          //client.println("Refresh: 5");  // refresh the page automatically every 5 sec
          //client.println();
          //Website:
          client.println("<html><title>Z21</title><body>Z21<br/>");
          //----------------------------------------------------------------------------------------------------          
          int firstPos = receivedText.indexOf("?");
          if (firstPos > -1) {
            byte lastPos = receivedText.indexOf(" ", firstPos);
            String theText = receivedText.substring(firstPos+3, lastPos); // 10 is the length of "?A="
            byte S88Pos = theText.indexOf("&S88=");
            #if defined(S88N)
              S88Module = theText.substring(S88Pos+5, theText.length()).toInt();
            #endif  
            byte Aip = theText.indexOf("&B=");
            byte Bip = theText.indexOf("&C=", Aip);
            byte Cip = theText.indexOf("&D=", Bip);
            byte Dip = theText.substring(Cip+3, S88Pos).toInt();
            Cip = theText.substring(Bip+3, Cip).toInt();
            Bip = theText.substring(Aip+3, Bip).toInt();
            Aip = theText.substring(0, Aip).toInt();
            ip[0] = Aip;
            ip[1] = Bip;
            ip[2] = Cip;
            ip[3] = Dip;
            client.println("-> RESET Z21");
            #if defined(S88N)
            if (EEPROM.read(EES88Moduls) != S88Module) {
              EEPROM.write(EES88Moduls, S88Module);
              SetupS88();
            }
            #endif
            if (EEPROM.read(EEip) != Aip)  
              EEPROM.write(EEip, Aip);
            if (EEPROM.read(EEip+1) != Bip)  
              EEPROM.write(EEip+1, Bip);
            if (EEPROM.read(EEip+2) != Cip)  
              EEPROM.write(EEip+2, Cip);
            if (EEPROM.read(EEip+3) != Dip)  
              EEPROM.write(EEip+3, Dip);
          }
          //----------------------------------------------------------------------------------------------------          
          client.print("<form method=get>IP:<input type=number min=0 max=254 name=A value=");
          client.println(ip[0]);
          client.print(">.<input type=number min=0 max=254 name=B value=");
          client.println(ip[1]);
          client.print(">.<input type=number min=0 max=254 name=C value=");
          client.println(ip[2]);
          client.print(">.<input type=number min=0 max=254 name=D value=");
          client.println(ip[3]);
          client.print("><br /> S88 8x Module: <input type=number min=0 max=62 name=S88 value=");
          #if defined(S88N)
            client.print(S88Module);
          #else
            client.print("-");
          #endif
          client.println("><br/><input type=submit></form></body></html>");
          break;
        }
        if (c == '\n') 
          currentLineIsBlank = true; // you're starting a new line
        else if (c != '\r') 
          currentLineIsBlank = false; // you've gotten a character on the current line
      }
    }
    client.stop();  // close the connection:
  }
}
#endif

//--------------------------------------------------------------------------------------------
void Ethreceive() {
  if(Udp.parsePacket() > 0) {  //packetSize
    addIPToSlot(Udp.remoteIP()[0], Udp.remoteIP()[1], Udp.remoteIP()[2], Udp.remoteIP()[3], 0);
    Udp.read(packetBuffer,UDP_TX_MAX_SIZE);  // read the packet into packetBufffer
    // send a reply, to the IP address and port that sent us the packet we received
    //  int header = (packetBuffer[3]<<8) + packetBuffer[2];
    //  int datalen = (packetBuffer[1]<<8) + packetBuffer[0];
    byte data[16]; 
    boolean ok = false;
    switch ((packetBuffer[3]<<8) + packetBuffer[2]) {  //header
    case LAN_GET_SERIAL_NUMBER:
      #if defined(DEBUG)
      Serial.println("GET_SERIAL_NUMBER");  
      #endif
      data[0] = 0xF5;  //Seriennummer 32 Bit (little endian)
      data[1] = 0x0A;
      data[2] = 0x00; 
      data[3] = 0x00;
      EthSend (0x08, LAN_GET_SERIAL_NUMBER, data, false, Z21bcNone);
      break; 
    case LAN_GET_HWINFO:
      #if defined(DEBUG)
      Serial.println("GET_HWINFO"); 
      #endif
      data[0] = HWTypeLSB;  //HwType 32 Bit
      data[1] = HWTypeMSB;
      data[2] = 0x00; 
      data[3] = 0x00;
      data[4] = FWVersionLSB;  //FW Version 32 Bit
      data[5] = FWVersionMSB;
      data[6] = 0x00; 
      data[7] = 0x00;
      EthSend (0x0C, LAN_GET_HWINFO, data, false, Z21bcNone);
      break;  
    case LAN_LOGOFF:
      #if defined(DEBUG)
      Serial.println("LOGOFF");
      #endif
      clearIPSlot(Udp.remoteIP()[0], Udp.remoteIP()[1], Udp.remoteIP()[2], Udp.remoteIP()[3]);
      //Antwort von Z21: keine
      break; 
      case (LAN_X_Header):
      switch (packetBuffer[4]) { //X-Header
      case LAN_X_GET_SETTING: 
        switch (packetBuffer[5]) {  //DB0
        case 0x21:
          #if defined(DEBUG)
          Serial.println("X_GET_VERSION"); 
          #endif
          data[0] = 0x63;
          data[1] = 0x21;
          data[2] = 0x30;   //X-Bus Version
          data[3] = 0x12;  //ID der Zentrale
          EthSend (0x09, LAN_X_Header, data, true, Z21bcNone);
          break;
        case 0x24:
          data[0] = 0x62;
          data[1] = 0x22;
          data[2] = Railpower;
          //Serial.print("X_GET_STATUS "); 
          EthSend (0x08, LAN_X_Header, data, true, Z21bcNone);
          break;
        case 0x80:
          #if defined(DEBUG)
          Serial.println("X_SET_TRACK_POWER_OFF");
          #endif
          setPower(csTrackVoltageOff);
          break;
        case 0x81:
          #if defined(DEBUG)
          Serial.println("X_SET_TRACK_POWER_ON");
          #endif
          setPower(csNormal);
          break;  
        }
        break;  //ENDE DB0
      case LAN_X_CV_READ:
        if (packetBuffer[5] == 0x11) {  //DB0
          #if defined(DEBUG)
          Serial.println("X_CV_READ"); 
          #endif
          byte CV_MSB = packetBuffer[6];
          byte CV_LSB = packetBuffer[7];
        }
        break;             
      case LAN_X_CV_WRITE:
        if (packetBuffer[5] == 0x12) {  //DB0
          #if defined(DEBUG)
          Serial.println("X_CV_WRITE"); 
          #endif
          byte CV_MSB = packetBuffer[6];
          byte CV_LSB = packetBuffer[7];
          byte value = packetBuffer[8];
          dcc.opsProgDirectCV(word(CV_MSB,CV_LSB),value); 

          data[0] = LAN_X_CV_RESULT; //X-Header 0x64
          data[1] = 0x14; //DB0
          data[2] = CV_MSB;
          data[3] = CV_LSB;
          data[4] = value;
          EthSend (0x0A, LAN_X_Header, data, true, 0x00);
        }
        break;
      case LAN_X_CV_POM: 
        if (packetBuffer[5] == 0x30) {  //DB0
          uint8_t Adr = ((packetBuffer[6] & 0x3F) << 8) + packetBuffer[7];
          uint8_t CVAdr = ((packetBuffer[8] & B11) << 8) + packetBuffer[9]; 
          byte value = packetBuffer[10];
          if ((packetBuffer[8] >> 2) == B111011) {
            #if defined(DEBUG)
            Serial.println("X_CV_POM_WRITE_BYTE"); 
            #endif
            dcc.opsProgramCV(Adr, CVAdr, value);  //set decoder 
          }
          else if ((packetBuffer[8] >> 2) == B111010 && value == 0) {
            #if defined(DEBUG)
            Serial.println("X_CV_POM_WRITE_BIT"); 
            #endif
          }
          else {
              #if defined(DEBUG)
              Serial.println("X_CV_POM_READ_BYTE"); 
              #endif
          }
        }
        else if (packetBuffer[5] == 0x31) {  //DB0
          #if defined(DEBUG)
          Serial.println("X_CV_POM_ACCESSORY"); 
          #endif
        }
        break;      
      case LAN_X_GET_TURNOUT_INFO:
        #if defined(DEBUG)
        Serial.print("X_GET_TURNOUT_INFO "); 
        #endif
        data[0] = 0x43;  //X-HEADER
        data[1] = packetBuffer[5]; //High
        data[2] = packetBuffer[6]; //Low
        if (dcc.getBasicAccessoryInfo((packetBuffer[5] << 8) + packetBuffer[6]) == true)
          data[3] = 2;  //active
        else data[3] = 1;  //inactive
        EthSend (0x09, LAN_X_Header, data, true, Z21bcAll_s);    //BC new 23.04. !!! (old = 0)
        break;             
      case LAN_X_SET_TURNOUT: {
        #if defined(DEBUG)
        Serial.println("X_SET_TURNOUT ");
        #endif
        //bool TurnOnOff = bitRead(packetBuffer[7],3);  //Spule EIN/AUS
        if (bitRead(packetBuffer[7],0) == 1)
          dcc.setBasicAccessoryPos((packetBuffer[5] << 8) + packetBuffer[6], true);
        else dcc.setBasicAccessoryPos((packetBuffer[5] << 8) + packetBuffer[6], false); 
        break;  
      }
      case LAN_X_SET_STOP:
        #if defined(DEBUG)
        Serial.println("X_SET_STOP");
        #endif
        setPower(csEmergencyStop);
        break;  
      case LAN_X_GET_LOCO_INFO:
        if (packetBuffer[5] == 0xF0) {  //DB0
          //Serial.print("X_GET_LOCO_INFO: ");
          //Antwort: LAN_X_LOCO_INFO  Adr_MSB - Adr_LSB
          dcc.getLocoStateFull(((packetBuffer[6] & 0x3F) << 8) + packetBuffer[7], false); 
        }
        break;  
      case LAN_X_SET_LOCO:
        if (packetBuffer[5] == LAN_X_SET_LOCO_FUNCTION) {  //DB0
          //LAN_X_SET_LOCO_FUNCTION  Adr_MSB        Adr_LSB            Type (EIN/AUS/UM)      Funktion
          dcc.setLocoFunc(word(packetBuffer[6] & 0x3F, packetBuffer[7]), packetBuffer[8] >> 5, packetBuffer[8] & B00011111); 
        }
        else {  //DB0
          //Serial.print("X_SET_LOCO_DRIVE ");
          if (packetBuffer[5] & B11 == 3)
            dcc.setSpeed128(word(packetBuffer[6] & 0x3F, packetBuffer[7]), packetBuffer[8]);  
          else if (packetBuffer[5] & B11 == 2)
            dcc.setSpeed28(word(packetBuffer[6] & 0x3F, packetBuffer[7]), packetBuffer[8]);  
          else dcc.setSpeed14(word(packetBuffer[6] & 0x3F, packetBuffer[7]), packetBuffer[8]);  
        }
        break;  
      case LAN_X_GET_FIRMWARE_VERSION:
        #if defined(DEBUG)
        Serial.println("X_GET_FIRMWARE_VERSION"); 
        #endif
        data[0] = 0xF3;
        data[1] = 0x0A;
        data[2] = FWVersionMSB;   //V_MSB
        data[3] = FWVersionLSB;  //V_LSB
        EthSend (0x09, LAN_X_Header, data, true, Z21bcNone);
        break;     
      }
      break; 
      case (LAN_SET_BROADCASTFLAGS): {
        addIPToSlot(Udp.remoteIP()[0], Udp.remoteIP()[1], Udp.remoteIP()[2], Udp.remoteIP()[3], getLocalBcFlag(packetBuffer[4] | (packetBuffer[5] << 8) | (packetBuffer[6] << 16) | (packetBuffer[7] << 24)));
        setPower(Railpower);  //Zustand Gleisspannung Antworten
        #if defined(DEBUG)
          Serial.print("SET_BROADCASTFLAGS: "); 
          Serial.println(getZ21BcFlag (addIPToSlot(Udp.remoteIP()[0], Udp.remoteIP()[1], Udp.remoteIP()[2], Udp.remoteIP()[3], 0x00)), HEX);  
          // 1=BC Power, Loco INFO, Trnt INFO; 2=BC Änderungen der Rückmelder am R-Bus
        #endif
        break;
      }
      case (LAN_GET_BROADCASTFLAGS): {
        unsigned long flag = getZ21BcFlag (addIPToSlot(Udp.remoteIP()[0], Udp.remoteIP()[1], Udp.remoteIP()[2], Udp.remoteIP()[3], 0x00));  
        data[0] = flag;
        data[1] = flag >> 8;
        data[2] = flag >> 16;
        data[3] = flag >> 24;
        EthSend (0x08, LAN_GET_BROADCASTFLAGS, data, false, Z21bcNone); 
        #if defined(DEBUG)
          Serial.print("GET_BROADCASTFLAGS: ");
          Serial.println(flag, HEX);
        #endif
        break;
      }
      case (LAN_GET_LOCOMODE):
      break;
      case (LAN_SET_LOCOMODE):
      break;
      case (LAN_GET_TURNOUTMODE):
      break;
      case (LAN_SET_TURNOUTMODE):
      break;
      case (LAN_RMBUS_GETDATA):
      #if defined(DEBUG)
      Serial.println("RMBUS_GETDATA"); 
      #endif
      #if defined(S88N)
        S88sendon = 'm';    //Daten werden gemeldet!
        notifyS88Data();
      #endif
      break;
      case (LAN_RMBUS_PROGRAMMODULE):
      break;
      case (LAN_SYSTEMSTATE_GETDATA):
      data[0] = 0x00;  //MainCurrent mA
      data[1] = 0x01;  //MainCurrent mA
      data[2] = 0x00;  //ProgCurrent mA
      data[3] = 0x00;  //ProgCurrent mA        
      data[4] = 0x00;  //FilteredMainCurrent
      data[5] = 0x01;  //FilteredMainCurrent
      data[6] = 0x00;  //Temperature
      data[7] = 0x01;  //Temperature
      data[8] = 0x0F;  //SupplyVoltage
      data[9] = 0x01;  //SupplyVoltage
      data[10] = 0x0F;  //VCCVoltage
      data[11] = 0x01;  //VCCVoltage
      data[12] = Railpower;  //CentralState
      data[13] = 0x00;  //CentralStateEx
      data[14] = 0x00;  //reserved
      data[15] = 0x00;  //reserved
      EthSend (0x14, LAN_SYSTEMSTATE_DATACHANGED, data, false, Z21bcNone);
      break;
      case (LAN_RAILCOM_GETDATA):
      break;
      case (LAN_LOCONET_FROM_LAN): {
        #if defined(DEBUG)
          Serial.println("LOCONET_FROM_LAN"); 
        #endif
        #if defined(LOCONET)
        byte LNdata[packetBuffer[0] - 0x04];  //n Bytes
        for (byte i = 0; i < (packetBuffer[0] - 0x04); i++) 
          LNdata[i] = packetBuffer[0x04+i];
        LNSendPacket (LNdata, packetBuffer[0] - 0x04);  
        //Melden an andere LAN-Client das Meldung auf LocoNet-Bus geschrieben wurde
        EthSend (packetBuffer[0], 0xA2, packetBuffer, false, Z21bcLocoNet_s);  //LAN_LOCONET_FROM_LAN
        #endif
        break;
      }
      case (LAN_LOCONET_DISPATCH_ADDR):
        #if defined(DEBUG)
          Serial.print("LOCONET_DISPATCH_ADDR ");
          Serial.println(word(packetBuffer[5],packetBuffer[4]));
        #endif
        #if defined(LOCONET)
          if (LNdispatch(packetBuffer[5],packetBuffer[4]) == true) {
            data[0] = packetBuffer[4];
            data[1] = packetBuffer[5];
            data[3] = dispatchSlot;
            EthSend (0x07, 0xA3, data, false, Z21bcNone);
          }
        #endif 
        break;
      case (LAN_LOCONET_DETECTOR):
      #if defined(DEBUG)
        Serial.println("LOCONET_DETECTOR Abfrage");   
      #endif
      break;
    default:
      #if defined(DEBUG)
        Serial.print("UNKNOWN_COMMAND 0x"); 
        Serial.println((packetBuffer[3]<<8) + packetBuffer[2], HEX);  //Header
      #endif
      data[0] = 0x61;
      data[1] = 0x82;
      EthSend (0x07, LAN_X_Header, data, true, Z21bcNone);
    }
  }
}

//--------------------------------------------------------------------------------------------
//Convert Z21 LAN BC flag to local stored flag
byte getLocalBcFlag (unsigned long flag) {
  byte outFlag = 0;
  if ((flag & Z21bcAll) != 0)
    outFlag |= Z21bcAll_s;
  if ((flag & Z21bcRBus) != 0) 
    outFlag |= Z21bcRBus_s;
  if ((flag & Z21bcSystemInfo) != 0)
    outFlag |= Z21bcSystemInfo_s;
  if ((flag & Z21bcNetAll) != 0)
    outFlag |= Z21bcNetAll_s;
  if ((flag & Z21bcLocoNet) != 0)
    outFlag |= Z21bcLocoNet_s;
  if ((flag & Z21bcLocoNetLocos) != 0)
    outFlag |= Z21bcLocoNetLocos_s;
  if ((flag & Z21bcLocoNetSwitches) != 0)
    outFlag |= Z21bcLocoNetSwitches_s;
  if ((flag & Z21bcLocoNetGBM) != 0) 
    outFlag |= Z21bcLocoNetGBM_s;
  return outFlag;  
}

//--------------------------------------------------------------------------------------------
//Convert local stored flag back into a Z21 Flag
unsigned long getZ21BcFlag (byte flag) {
  unsigned long outFlag = 0;
  if ((flag & Z21bcAll_s) != 0)
    outFlag |= Z21bcAll;
  if ((flag & Z21bcRBus_s) != 0)
    outFlag |= Z21bcRBus;
  if ((flag & Z21bcSystemInfo_s) != 0)
    outFlag |= Z21bcSystemInfo;
  if ((flag & Z21bcNetAll_s) != 0)
    outFlag |= Z21bcNetAll;
  if ((flag & Z21bcLocoNet_s) != 0)
    outFlag |= Z21bcLocoNet;
  if ((flag & Z21bcLocoNetLocos_s) != 0)
    outFlag |= Z21bcLocoNetLocos;
  if ((flag & Z21bcLocoNetSwitches_s) != 0)
    outFlag |= Z21bcLocoNetSwitches;
  if ((flag & Z21bcLocoNetGBM_s) != 0)    
    outFlag |= Z21bcLocoNetGBM;
  return outFlag;
}

//--------------------------------------------------------------------------------------------
void EthSend (unsigned int DataLen, unsigned int Header, byte *dataString, boolean withXOR, byte BC) {
    IPAddress IPout = Udp.remoteIP();
    for (byte i = 0; i < maxIP; i++) {
      if (BC == 0 || (ActIP[i].time > 0 && BC & ActIP[i].BCFlag != 0)) {    //Boradcast & Noch aktiv

      if (BC == 0)
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());    //no Broadcast
      else {
        IPout[0] = ActIP[i].ip0;
        IPout[1] = ActIP[i].ip1;
        IPout[2] = ActIP[i].ip2;
        IPout[3] = ActIP[i].ip3;
        Udp.beginPacket(IPout, Udp.remotePort());    //Broadcast
      }
      //--------------------------------------------        
      Udp.write(DataLen & 0xFF);
      Udp.write(DataLen >> 8);
      Udp.write(Header & 0xFF);
      Udp.write(Header >> 8);
    
      unsigned char XOR = 0;
      byte ldata = DataLen-5;  //Ohne Length und Header und XOR
      if (!withXOR)    //XOR vorhanden?
        ldata++;
      for (byte i = 0; i < (ldata); i++) {
        XOR = XOR ^ *dataString;
        Udp.write(*dataString);
        dataString++;
      }
      if (withXOR)
        Udp.write(XOR);
      //--------------------------------------------
      Udp.endPacket();
      if (BC == 0)  //END when no BC
        return;
    }
  }
}

//-------------------------------------------------------------- 
//--------------------------------------------------------------
void notifyLokAll(uint16_t Adr, uint8_t Steps, uint8_t Speed, uint8_t F0, uint8_t F1, uint8_t F2, uint8_t F3, bool bc) {
  #if defined(LOCONET)
    LNSetLocoStatus(LNGetLocoSlot(Adr >> 8, Adr & 0xFF),Speed, F0, F1);
  #endif
    
  #if defined(XPRESSNET)  
  if (XNetReturnLocoInfo == true) {
    XNetReturnLocoInfo = false;
    Serial.println("Ask Speed");
    XpressNet.SetLocoInfo(XNetUserOps, 0x00, F0, F1); //UserOps,Speed,F0,F1
    return;
  }
  if (XNetReturnLocoFkt == true) {
    XNetReturnLocoFkt = false;
    XpressNet.SetFktStatus(XNetUserOps, F2, F3); //Fkt4, Fkt5
    return;
  }
  #endif
  
  byte data[9]; 
  data[0] = LAN_X_LOCO_INFO;  //0xEF X-HEADER
  data[1] = (Adr >> 8) & 0x3F;
  data[2] = Adr & 0xFF;
  if (Steps == 3) //nicht vorhanden!
    data[3] = 4;
  else data[3] = Steps;  
  data[4] = Speed;
  data[5] = F0;    //F0, F4, F3, F2, F1
  data[6] = F1;    //F5 - F12; Funktion F5 ist bit0 (LSB)
  data[7] = F2;  //F13-F20
  data[8] = F3;  //F21-F28
  if (bc == false)  //kein BC
    EthSend (14, LAN_X_Header, data, true, Z21bcNone);  //Send Power und Funktions to all active Apps
  else EthSend (14, LAN_X_Header, data, true, Z21bcAll_s | Z21bcNetAll_s);  //Send Power und Funktions to all active Apps 
}
//-------------------------------------------------------------- 
void notifyTrnt(uint16_t Adr, bool State) {
  byte data[4];
  data[0] = LAN_X_TURNOUT_INFO;  //0x43 X-HEADER
  data[1] = Adr >> 8;   //High
  data[2] = Adr & 0xFF; //Low
  data[3] = State+1;
//  if (State == true)
//    data[3] = 2;
//  else data[3] = 1;  
  EthSend (0x09, LAN_X_Header, data, true, Z21bcAll_s); 
}

//--------------------------------------------------------------------------------------------
#if defined(DEBUG)
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
#endif

