//Z21 LAN and WiFi 
//Kommunikation mit der Z21 Library!
//
//Copyright (c) by Philipp Gahtow, year 2015
/*
 * Add change für WLAN für multi Paket - meherer Abfragen vom Client
 * Add change via LAN für multi Paket
 */
//----------------------------------------------
#if defined(LAN) || defined(WIFI)

// buffers for receiving and sending data
#define Z21_UDP_TX_MAX_SIZE 20  //--> LocoNet DATA has 20 Byte!

#if defined(LAN)
//----------------------------------------------
#define Z21_Eth_offset WLANmaxIP   //shift LAN client behind WiFi client

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

#if defined(WIFI)
byte outData[Z21_UDP_TX_MAX_SIZE];    //store received Serial to send via UDP
byte outDcount = 0;  //position writing out data
byte sendTOO = 0xFF;  //memIP to whom to send the data
#endif

//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
#if defined(WIFI)
void WLANSetup() {
    digitalWrite(DCCLed, HIGH);   //show Waiting for WiFi Modul
    WLAN.begin(WIFISerialBaud);  //UDP to Serial Kommunikation
    byte b0 = 0;
    byte b1 = 0;
    #if defined(DEBUG)
    Debug.print(WIFISerialBaud);
    Debug.print(F(" start WIFI.."));
    #endif
    byte timeout = 20;  //try to init at least
    do {
      #if defined(DEBUG)
      Debug.print(".");
      #endif
      #if defined(S88N)
      WLAN.write(S88Module);  
      #else 
      WLAN.write(0xFE);
      #endif
      WLAN.print(F("S88"));
      delay(800);   //wait for ESP8266 to receive!
      while (WLAN.available() > 0) {
         b0 = b1;
         b1 = WLAN.read();
      }
      timeout--;
    }  
    while (b0 != 'O' && b1 != 'K' && timeout > 0);
    #if defined(DEBUG)
    if (timeout == 0)
      Debug.println("FAIL");
    else Debug.println("OK");
    #endif   
    digitalWrite(DCCLed, LOW);  //Wifi detecting finish!
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
    IPAddress remote = Udp.remoteIP();
    byte packetBuffer[packetSize];
    /*byte len = */Udp.read(packetBuffer,packetSize); // Z21_UDP_TX_MAX_SIZE);  // read the packet into packetBufffer
    if (packetSize == packetBuffer[0]) { //normal:
      #if defined(Z21DEBUG)
      Debug.print(Z21addIP(remote[0], remote[1], remote[2], remote[3]) + Z21_Eth_offset);
      Debug.print(" Z21 RX: ");
      for (byte i = 0; i < packetBuffer[0]; i++) {
        Debug.print(packetBuffer[i], HEX);
        Debug.print(" ");
      }
      Debug.println();
      #endif
      z21.receive(Z21addIP(remote[0], remote[1], remote[2], remote[3]) + Z21_Eth_offset, packetBuffer);
    }
    else {  //kombiniertes UDP Paket:
      byte readpos = 0;   //position des aktuellen Paket
      //durchlaufe alle Pakete:
      do {    
        byte pBuffer[packetBuffer[readpos]];   //array of packet length
        for (byte i = 0; i < packetBuffer[readpos]; i++)    //fill up array with packet
          pBuffer[i] = packetBuffer[readpos + i];
        #if defined(Z21DEBUG)
        Debug.print(Z21addIP(remote[0], remote[1], remote[2], remote[3]) + Z21_Eth_offset);
        Debug.print("-Z21 RX: ");
        for (byte i = 0; i < pBuffer[0]; i++) {
          Debug.print(pBuffer[i], HEX);
          Debug.print(" ");
        }
        Debug.println();
        #endif
        z21.receive(Z21addIP(remote[0], remote[1], remote[2], remote[3]) + Z21_Eth_offset, pBuffer);
        readpos = packetBuffer[readpos] + readpos;  //bestimme position des nächsten Paket
      }
      while(readpos < packetSize);
    }
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
        //kombinierte UDP Paket:
        if (outData[0] == 0) {  //Double Packet - check 2. Bit! 
          outData[0] = sendTOO;  //lese Länge des Packet
          sendTOO = outData[1]; //read back Stored client!
          outData[1] = 0;   //2. Bit der länge ist immer "0"
          outDcount++;  //goto next Bit
        }
        if (outData[0] <= outDcount) {  //1. Byte gibt länge der Daten an!
          if (sendTOO <= WLANmaxIP) {     //Ziel in range?
              //read Data:
            #if defined(Z21DEBUG)  
            Debug.print(sendTOO);
            Debug.print("-Z21_RX: ");
            for (byte i = 0; i < outData[0]; i++) {
              Debug.print(outData[i], HEX);
              Debug.print(" ");
            }
            Debug.println(F(" Z21 WLAN ok"));  //Accept
            #endif
            z21.receive(sendTOO, outData);
            outData[1] = sendTOO;  //Store Client that send the DATA
          }
          #if defined(Z21DEBUG)
          else {
            Debug.print(F("Z21 EE "));  //Fail
            Debug.println(sendTOO, HEX);
          }
          #endif
          outDcount = 0;
          sendTOO = 0xFF;
        }
        else if ((outDcount >= Z21_UDP_TX_MAX_SIZE) || (sendTOO == 'E' && outData[0] == 'E') || (outData[outDcount-2] == 'E' && outData[outDcount-1] == 'E')) { 
          #if defined(Z21DEBUG)
          Debug.write(sendTOO);  //Fail?
          for (byte i = 0; i < outDcount; i++)
              Debug.write(outData[i]);
          Debug.println(F(" FLUSH!"));
          #endif
          outDcount = 0;  //reset read buffer
          sendTOO = 0xFF;
          //WLAN.flush();   //clear
        }
        else if ((sendTOO == 'O' && outData[0] == 'K') || (outData[outDcount-2] == 'O' && outData[outDcount-1] == 'K')) {  //keine valied Data!
          #if defined(Z21DEBUG)
          Debug.println(F("Z21 OK"));
          #endif
          outDcount = 0;  //reset read buffer
          sendTOO = 0xFF;
        }
        //S88 Module config:
        else if (outData[0] == 'S' && outData[1] == '8' && outData[2] == '8') {
          #if defined(S88N)
          if (sendTOO <= S88MAXMODULE) {  // && sendTOO >= 0
            S88Module = sendTOO;
            #if defined(REPORT)
            Debug.print(F("Set S88 Module: "));
            Debug.println(S88Module);
            #endif
            if (EEPROM.read(EES88Moduls) != S88Module) {
              EEPROM.write(EES88Moduls, S88Module);
              SetupS88();
              WLANSetup();
            }
          }
          #else
          WLAN.write(0XFE);  //kein S88 aktiv!
          WLAN.print(F("S88"));
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
  if((currentMillis - IPpreviousMillis) >= IPinterval) {
    IPpreviousMillis = currentMillis;   
    for (byte i = 0; i < storedIP; i++) {
        if (mem[i].time > 0) {
          mem[i].time--;    //Zeit herrunterrechnen
          #if defined(Z21DEBUG)
          if (mem[i].time == 0) {
            Debug.print(i);
            Debug.println(F(" LAN client off"));
          }
          #endif
        }
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
  #if defined(Z21DEBUG)  
    Debug.print(F("z21 PW: "));
  #endif          
  if (Railpower != State)
    globalPower(State);
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
  
  #if defined(LOCONET) && !defined(LnSLOTSRV)
  if (fkt >= 0 && fkt <= 4) {
    byte DIRF = dcc.getFunktion0to4(Adr) | (!dcc.getLocoDir(Adr) << 5);
    sendLNDIRF(Adr,  DIRF);
  }
  if (fkt >= 5 && fkt <= 8)
    sendLNSND(Adr, dcc.getFunktion5to8(Adr));
  #endif
}

//--------------------------------------------------------------------------------------------
void notifyz21LocoSpeed(uint16_t Adr, uint8_t speed, uint8_t steps)
{
  #if defined(LOCONET) && !defined(LnSLOTSRV)
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
  Debug.print(Adr);
  Debug.print("-");
  Debug.println(dcc.getBasicAccessoryInfo(Adr));
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
    Debug.println(F("LOCONET_FROM_LAN")); 
  #endif
  
  LNSendPacket (data, length);  
}
//--------------------------------------------------------------------------------------------
#if defined(Z21DEBUG)
void notifyz21LNdetector(uint8_t typ, uint16_t Adr) {
  Debug.println(F("LAN_LOCONET_DETECTOR")); 
}
#endif

#endif

//--------------------------------------------------------------------------------------------
//read out the current of the booster main - in testing!
#if defined(BOOSTER_INT_MAINCURRENT)
uint16_t notifyz21MainCurrent()
{
  unsigned int val = analogRead(VAmpIntPin);
  if (val > 511)
    val = val - 511;
  else val = 512 - val;
  return val;  
}
#endif

//--------------------------------------------------------------------------------------------
void notifyz21CVREAD(uint8_t cvAdrMSB, uint8_t cvAdrLSB)
{
  //no Hardware that support reading connected!
}

//--------------------------------------------------------------------------------------------
uint8_t notifyz21CVWRITE(uint8_t cvAdrMSB, uint8_t cvAdrLSB, uint8_t value)
{
  #if defined(Z21DEBUG)

  #if defined(BOOSTER_INT_MAINCURRENT)
  pinMode(VAmpIntPin, INPUT);
  unsigned int val = analogRead(VAmpIntPin);
  if (val > 511)
    Debug.print(val - 511);
  else Debug.print(512 - val);  
  #endif
  
  Debug.print(F("Z21 Prog Direct: "));
  Debug.print(word(cvAdrMSB, cvAdrLSB));
  Debug.print(" - ");
  Debug.println(value);
  #endif
  
  dcc.opsProgDirectCV(word(cvAdrMSB, cvAdrLSB),value);

  #if defined(BOOSTER_INT_MAINCURRENT)
  for (byte i = 1; i < 100; i++) {
    unsigned int readval = analogRead(VAmpIntPin);
    if (((readval + 2) >= val && val > 511) || ((readval - 2) <= val && val < 512)) {
      if (readval > 511)
        Debug.print(readval - 511);
      else Debug.print(512 - readval); 
      Debug.println(" ACK");
    }
    val = readval;
  }
  #endif
  
  return value;
}

//--------------------------------------------------------------------------------------------
void notifyz21CVPOMWRITEBYTE(uint16_t Adr, uint16_t cvAdr, uint8_t value)
{
  #if defined(Z21DEBUG)
  Debug.print(F("Z21 Prog POM write: "));
  Debug.print(Adr);
  Debug.print(" set ");
  Debug.print(cvAdr);
  Debug.print(" - ");
  Debug.println(value);
  #endif
  dcc.opsProgramCV(Adr, cvAdr, value);  //set decoder
 // dcc.opsPOMreadCV(Adr, cvAdr);  //get decoder value 
}

//--------------------------------------------------------------------------------------------
void notifyz21CVPOMREADBYTE (uint16_t Adr, uint16_t cvAdr)
{
  #if defined(Z21DEBUG)
  Debug.print(F("Z21 Prog POM read: "));
  Debug.print(Adr);
  Debug.print(" get ");
  Debug.println(cvAdr);
  #endif
  dcc.opsPOMreadCV(Adr, cvAdr);  //get decoder value
  return 1;
}

//--------------------------------------------------------------------------------------------
void notifyz21EthSend(uint8_t client, uint8_t *data) 
{
  #if defined(Z21DEBUG)
  Debug.print(client);
  Debug.print(F(" Z21 TX: "));
  for (byte i = 0; i < data[0]; i++) {
    Debug.print(data[i], HEX);
    Debug.print(" ");
  }
  #endif
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

