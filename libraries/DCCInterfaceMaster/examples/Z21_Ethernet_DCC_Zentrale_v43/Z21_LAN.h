//Z21 LAN and WiFi 
//Kommunikation mit der Z21 Library!
//
//Copyright (c) by Philipp Gahtow, year 2015
//----------------------------------------------
#if defined(LAN) || defined(WIFI)

// buffers for receiving and sending data
#define Z21_UDP_TX_MAX_SIZE 20  //--> LocoNet DATA has 20 Byte!

#if defined(LAN)
//----------------------------------------------
unsigned char packetBuffer[Z21_UDP_TX_MAX_SIZE]; //buffer to hold incoming packet

#define Z21_Eth_offset LANmaxIP+1   //shift LAN client behind WiFi client

//byte Z21client = 0; //Absender der letzten Nachricht

typedef struct    //Rückmeldung des Status der Programmierung
{
  byte IP0; //client IP-Adresse
  byte IP1;
  byte IP2;
  byte IP3;
  byte time;  //aktive Zeit
} listofIP;
listofIP mem[LANmaxIP];
byte storedIP = 0;  //speicher für IPs

unsigned long IPpreviousMillis = 0;       // will store last time of IP decount updated
//----------------------------------------------
#endif

byte outData[Z21_UDP_TX_MAX_SIZE];    //store received Serial to send via UDP
byte outDcount = 0;  //position writing out data
byte sendTOO = 0xFF;  //memIP to whom to send the data

//--------------------------------------------------------------------------------------------
//Sende Anzahl S88 Module to ESP
#if defined(S88N)
void WLANnotifyS88Module() {
  #if defined(WIFI)
  WLAN.write(S88Module);
  WLAN.print("S88");
  #endif
}
#endif

//--------------------------------------------------------------------------------------------
#if defined(WIFI)
void WLANSetup() {
    WLAN.begin(115200);  //UDP to Serial Kommunikation
    #if defined(S88N)
    WLANnotifyS88Module();
    #endif
}
#endif

//--------------------------------------------------------------------------------------------
#if defined(LAN)
byte Z21addIP (byte ip0, byte ip1, byte ip2, byte ip3) {
  //suche ob IP schon vorhanden?
  for (byte i = 0; i < storedIP; i++) {
    if (mem[i].IP0 == ip0 && mem[i].IP1 == ip1 && mem[i].IP2 == ip2 && mem[i].IP3 == ip3) {
      mem[i].time = ActTimeIP; //setzte Zeit
      return i+1;
    }
  }
  if (storedIP >= LANmaxIP) {
    for (byte i = 0; i < storedIP; i++) {
      if (mem[i].time == 0) { //Abgelaufende IP, dort eintragen!
         mem[i].IP0 = ip0;
         mem[i].IP1 = ip1;
         mem[i].IP2 = ip2;
         mem[i].IP3 = ip3;
         mem[i].time = ActTimeIP; //setzte Zeit
         return i+1;
      }
    }
    return 0; //keine freien IPs (never reach here!)
  }
  mem[storedIP].IP0 = ip0;
  mem[storedIP].IP1 = ip1;
  mem[storedIP].IP2 = ip2;
  mem[storedIP].IP3 = ip3;
  mem[storedIP].time = ActTimeIP; //setzte Zeit
  storedIP++;
  return storedIP;  
}
#endif

//--------------------------------------------------------------------------------------------
void Z21LANreceive () {
  //receive LAN data:
  #if defined(LAN)
  byte packetSize = Udp.parsePacket();
  if(packetSize) {  //packetSize
    Udp.read(packetBuffer,Z21_UDP_TX_MAX_SIZE);  // read the packet into packetBufffer
    IPAddress remote = Udp.remoteIP();
    if (packetSize == packetBuffer[0]) 
      z21.receive(Z21addIP(remote[0], remote[1], remote[2], remote[3]) + Z21_Eth_offset, packetBuffer);
  }
  #endif

  //receive WLAN data:
  #if defined(WIFI)
  if (WLAN.available() > 0) {  //Senden UDP
      outData[outDcount] = WLAN.read(); //read
      //Serial.write(outData[outDcount]);
      if (sendTOO == 0xFF)
        sendTOO = outData[outDcount];  //Ziel IP/client read out
      else {
        outDcount++;
        if (outData[0] <= outDcount) {  //1. Byte gibt länge der Daten an!
          if (sendTOO <= WLANmaxIP) {     //Ziel in range?
              //read Data:
            z21.receive(sendTOO, outData);
            #if defined(Z21DEBUG)  
            Debug.print(sendTOO);
            Debug.print(" Z21 RX: ");
            for (byte i = 0; i < outData[0]; i++) {
              Debug.print(outData[i], HEX);
              Debug.print(" ");
            }
            Debug.println(" Z21 WLAN ok");  //Accept
            #endif
          }
          #if defined(Z21DEBUG)
          else Debug.println("Z21 EE");  //Fail
          #endif
          outDcount = 0;
          sendTOO = 0xFF;
        }
        if ((outDcount >= Z21_UDP_TX_MAX_SIZE) || (sendTOO == 'E' && outData[0] == 'E')) { 
          #if defined(Z21DEBUG)
          Debug.write(sendTOO);  //Fail?
          Debug.write(outData[0]);
          Debug.println(" FLUSH!");
          #endif
          outDcount = 0;  //reset read buffer
          sendTOO = 0xFF;
          //WLAN.flush();   //clear
        }
        if (sendTOO == 'O' && outData[0] == 'K') {  //keine valied Data!
          #if defined(Z21DEBUG)
          Debug.println("Z21 OK");
          #endif
          outDcount = 0;  //reset read buffer
          sendTOO = 0xFF;
        }
        //S88 Module config:
        if (outData[0] == 'S' && outData[1] == '8' && outData[2] == '8') {
          #if defined(S88N)
          if (sendTOO <= S88MAXMODULE && sendTOO >= 0) {
            S88Module = sendTOO;
            #if defined(DEBUG)
            Debug.print("Set S88 Module: ");
            Debug.println(S88Module);
            #endif
            if (EEPROM.read(EES88Moduls) != S88Module) {
              EEPROM.write(EES88Moduls, S88Module);
              SetupS88();
              WLANnotifyS88Module();
            }
          }
          #else
          WLAN.write(0XFE);  //kein S88 aktiv!
          WLAN.print("S88");
          #endif
          outDcount = 0;   //reset read buffer
          sendTOO = 0xFF;
        }
      }
  }
  #endif
  #if defined(LAN)
  //Nutzungszeit IP's bestimmen
  unsigned long currentMillis = millis();
  if(currentMillis - IPpreviousMillis > interval) {
    IPpreviousMillis = currentMillis;   
    for (byte i = 0; i < storedIP; i++) {
        if (mem[i].time > 0) 
          mem[i].time--;    //Zeit herrunterrechnen
    }
  }
  #endif
}

//--------------------------------------------------------------------------------------------
#if defined(S88N)
void notifyz21S88Data() {
  S88sendon = 'i';
  notifyS88Data();
}
#endif

//--------------------------------------------------------------------------------------------
void notifyz21RailPower(uint8_t State)
{
  if (Railpower != State)
    setPower(State);
}

//--------------------------------------------------------------------------------------------
void notifyz21getLocoState(uint16_t Adr, bool bc)
{
  dcc.getLocoStateFull(Adr, bc);
}

//--------------------------------------------------------------------------------------------
void notifyz21LocoFkt(uint16_t Adr, uint8_t state, uint8_t fkt)
{
  dcc.setLocoFunc(Adr, state, fkt); 
  dcc.getLocoStateFull(Adr, false);      //request for other devices
  #if defined(LOCONET) && !defined(LNMaster)
  if (fkt >= 0 && fkt <= 4) {
    byte DIRF = dcc.getFunktion0to4(Adr) | (dcc.getLocoDir(Adr) << 5);
    sendLNDIRF(Adr,  DIRF);
  }
  if (fkt >= 5 && fkt <= 8)
    sendLNSND(Adr, dcc.getFunktion5to8(Adr));
  #endif
}

//--------------------------------------------------------------------------------------------
void notifyz21LocoSpeed(uint16_t Adr, uint8_t speed, uint8_t steps)
{
  #if defined(LOCONET) && !defined(LNMaster)
  sendLNSPD(Adr, speed);
  #endif
  
  switch (steps) {
    case 14: dcc.setSpeed14(Adr, speed); break;
    case 28: dcc.setSpeed28(Adr, speed); break;
    default: dcc.setSpeed128(Adr, speed); 
  }
  dcc.getLocoStateFull(Adr, false);      //request for other devices
}

//--------------------------------------------------------------------------------------------
void notifyz21Accessory(uint16_t Adr, bool state, bool active)
{
  dcc.setBasicAccessoryPos(Adr, state, active);
  #if defined(LOCONET)
  LNsetTrnt(Adr, state, active);
  #endif
}

//--------------------------------------------------------------------------------------------
uint8_t notifyz21AccessoryInfo(uint16_t Adr)
//return state of the Address (left/right = true/false)
{
  #if defined(Z21DEBUG)
  Debug.print("Z21 GetAccInfo: ");
  Debug.println(Adr);
  #endif
  return dcc.getBasicAccessoryInfo(Adr);
}

#if defined(LOCONET)
//--------------------------------------------------------------------------------------------
uint8_t notifyz21LNdispatch(uint8_t Adr2, uint8_t Adr)
//return the Slot that was dispatched, 0xFF at error!
{
  #if defined(Z21DEBUG)
  Debug.print("Z21 LNdispatch: ");
  Debug.println(word(Adr2,Adr));
  #endif
  return LNdispatch(Adr2, Adr);
}
//--------------------------------------------------------------------------------------------
void notifyz21LNSendPacket(uint8_t *data, uint8_t length)
{
  #if defined(Z21DEBUG)
    Debug.println("LOCONET_FROM_LAN"); 
  #endif
  
  LNSendPacket (data, length);  
}
//--------------------------------------------------------------------------------------------
void notifyz21LNdetector(uint8_t typ, uint16_t Adr) {
  #if defined(Z21DEBUG)
    Debug.println("LAN_LOCONET_DETECTOR request"); 
  #endif
}
#endif

//--------------------------------------------------------------------------------------------
void notifyz21CVREAD(uint8_t cvAdrMSB, uint8_t cvAdrLSB)
{
  //no Hardware that support reading connected!
}

//--------------------------------------------------------------------------------------------
void notifyz21CVWRITE(uint8_t cvAdrMSB, uint8_t cvAdrLSB, uint8_t value)
{
  dcc.opsProgDirectCV(word(cvAdrMSB, cvAdrLSB),value);
}

//--------------------------------------------------------------------------------------------
void notifyz21CVPOMWRITEBYTE(uint16_t Adr, uint16_t cvAdr, uint8_t value)
{
  dcc.opsProgramCV(Adr, cvAdr, value);  //set decoder 
}

//--------------------------------------------------------------------------------------------
void notifyz21EthSend(uint8_t client, uint8_t *data) 
{
  #if defined(Z21DEBUG)
  Debug.print(client);
  Debug.print(" Z21 TX: ");
  for (byte i = 0; i < data[0]; i++) {
    Debug.print(data[i], HEX);
    Debug.print(" ");
  }
  #endif;
  if (client == 0) { //all stored 
    #if defined(LAN)
    for (byte i = 0; i < storedIP; i++) {
      if (mem[i].time > 0) {  //noch aktiv?
        IPAddress ip(mem[i].IP0, mem[i].IP1, mem[i].IP2, mem[i].IP3);
        Udp.beginPacket(ip, Udp.remotePort());    //Broadcast
        Udp.write(data, data[0]);
        Udp.endPacket();
      }
    }
    #endif
    
    #if defined(WIFI)
    WLAN.write(client);
    WLAN.write(data, data[0]);
    #endif
  }
  else {
    #if defined(LAN)
    if (client < Z21_Eth_offset) {  //Prüfe über Offset ob WiFi or LAN
    #endif  
      #if defined(WIFI)
      WLAN.write(client);
      WLAN.write(data, data[0]);
      #endif
    #if defined(LAN)
    }
    else {
      byte cl = client - Z21_Eth_offset - 1;  //senden ohne Offset!
      IPAddress ip(mem[cl].IP0, mem[cl].IP1, mem[cl].IP2, mem[cl].IP3);
      Udp.beginPacket(ip, Udp.remotePort());    //no Broadcast
      Udp.write(data, data[0]);
      Udp.endPacket();
    }
    #endif
  }
}

//---------------------------------
#endif

