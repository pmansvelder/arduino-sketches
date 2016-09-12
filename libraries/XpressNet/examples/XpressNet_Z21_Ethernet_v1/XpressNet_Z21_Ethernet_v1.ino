/*
    Z21 Ethernet Emulation für die App-Steuerung via Smartphone über XpressNet.
    
    Version 1
 */
 
#include <XpressNet.h> 
XpressNetClass XpressNet;

#include <EEPROM.h>
#define EEip 10    //Startddress im EEPROM für die IP
#define EEXNet 9   //Adresse im XNet-Bus

#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = { 0xFE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 188, 111);

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
// (port 80 is default for HTTP):

EthernetServer server(80);

#define localPort 21105      // local port to listen on
#define XNetTxRxPin 9    //Send/Receive Pin MAX

#define ResetPin A1  //Reset Pin bei Neustart betätigen um Standard IP zu setzten!

// XpressNet address: must be in range of 1-31; must be unique. Note that some IDs
// are currently used by default, like 2 for a LH90 or LH100 out of the box, or 30
// for PC interface devices like the XnTCP.
byte XNetAddress = 30;    //Adresse im XpressNet
byte XBusVer = 0x30;      //Version XNet-Bus (default 3.0)

// buffers for receiving and sending data
unsigned char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,

#define maxIP 10        //Speichergröße für IP-Adressen
#define ActTimeIP 20    //Aktivhaltung einer IP für (sec./2)
#define interval 2000   //interval at milliseconds
struct TypeActIP {
  byte ip0;    // Byte IP
  byte ip1;    // Byte IP
  byte ip2;    // Byte IP
  byte ip3;    // Byte IP
  byte time;  //Zeit
};
TypeActIP ActIP[maxIP];    //Speicherarray für IPs
long previousMillis = 0;        // will store last time of IP decount updated

//#include <SoftwareSerial.h>
//SoftwareSerial Debug(6, 5); // RX, TX

//--------------------------------------------------------------------------------------------
void setup() {
 // //Debug.begin(115200); 
 // //Debug.println("Z21"); 
   pinMode(ResetPin, INPUT);  
   digitalWrite(ResetPin, HIGH);  //PullUp  
   delay(100);
   
   if (digitalRead(ResetPin) == LOW || EEPROM.read(EEXNet) < 32) {
     XNetAddress = EEPROM.read(EEXNet);
   }
   else {  
      EEPROM.write(EEXNet, XNetAddress);
      EEPROM.write(EEip, ip[0]);
      EEPROM.write(EEip+1, ip[1]);
      EEPROM.write(EEip+2, ip[2]);
      EEPROM.write(EEip+3, ip[3]);
    }
  ip[0] = EEPROM.read(EEip);
  ip[1] = EEPROM.read(EEip+1);
  ip[2] = EEPROM.read(EEip+2);
  ip[3] = EEPROM.read(EEip+3);

//  //Debug.println(ip);
 
  // start the Ethernet and UDP:
  Ethernet.begin(mac,ip);  //IP and MAC Festlegung
//  server.begin();    //HTTP Server
  Udp.begin(localPort);  //UDP Z21 Port
    
  XpressNet.start(XNetAddress, XNetTxRxPin);    //Initialisierung XNet
  
  clearIPSlots();  //löschen gespeicherter aktiver IP's
}

//--------------------------------------------------------------------------------------------
void notifyXNetVersion(uint8_t Version, uint8_t ID ) {
  XBusVer = Version;
}

/*
//--------------------------------------------------------------------------------------------
void notifyXNetStatus(uint8_t LedState )
{
}
*/



//--------------------------------------------------------------------------------------------
void loop() {
  
  XpressNet.receive();  //Check for XpressNet
 
  Ethreceive();    //Read Data on UDP Port
  
  XpressNet.receive();  //Check for XpressNet
  
  Webconfig();    //Webserver for Configuration

  unsigned long currentMillis = millis();
  
  if(currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;   
    for (int i = 0; i < maxIP; i++) {
      if (ActIP[i].time > 0) {
        ActIP[i].time--;    //Zeit herrunterrechnen
      }
      else {
        //clear IP DATA
        ActIP[i].ip0 = 0;
        ActIP[i].ip1 = 0;
        ActIP[i].ip2 = 0;
        ActIP[i].ip3 = 0;
        ActIP[i].time = 0;
      }
    } 
  }
}

//--------------------------------------------------------------------------------------------
void clearIPSlots() {
  for (int i = 0; i < maxIP; i++) {
    ActIP[i].ip0 = 0;
    ActIP[i].ip1 = 0;
    ActIP[i].ip2 = 0;
    ActIP[i].ip3 = 0;
    ActIP[i].time = 0;
  }
}

//--------------------------------------------------------------------------------------------
void addIPToSlot (byte ip0, byte ip1, byte ip2, byte ip3) {
  byte Slot = maxIP;
  for (int i = 0; i < maxIP; i++) {
    if (ActIP[i].ip0 == ip0 && ActIP[i].ip1 == ip1 && ActIP[i].ip2 == ip2 && ActIP[i].ip3 == ip3) {
      ActIP[i].time = ActTimeIP;
      return;
    }
    else if (ActIP[i].time == 0 && Slot == maxIP)
      Slot = i;
  }
  ActIP[Slot].ip0 = ip0;
  ActIP[Slot].ip1 = ip1;
  ActIP[Slot].ip2 = ip2;
  ActIP[Slot].ip3 = ip3;
  ActIP[Slot].time = ActTimeIP;
  notifyXNetPower(XpressNet.getPower());
}



//--------------------------------------------------------------------------------------------
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
          client.println();
          //Website:
          client.println("<html><head><title>Z21</title></head><body>");
          client.println("<h1>Z21</h1><br />");
//----------------------------------------------------------------------------------------------------          
          int firstPos = receivedText.indexOf("?");
          if (firstPos > -1) {
            client.println("-> accept changes after RESET!");
            byte lastPos = receivedText.indexOf(" ", firstPos);
            String theText = receivedText.substring(firstPos+3, lastPos); // 10 is the length of "?A="
            byte XNetPos = theText.indexOf("&XNet=");
            XNetAddress = theText.substring(XNetPos+6, theText.length()).toInt();
            byte Aip = theText.indexOf("&B=");
            byte Bip = theText.indexOf("&C=", Aip);
            byte Cip = theText.indexOf("&D=", Bip);
            byte Dip = theText.substring(Cip+3, XNetPos).toInt();
            Cip = theText.substring(Bip+3, Cip).toInt();
            Bip = theText.substring(Aip+3, Bip).toInt();
            Aip = theText.substring(0, Aip).toInt();
            ip[0] = Aip;
            ip[1] = Bip;
            ip[2] = Cip;
            ip[3] = Dip;
            if (EEPROM.read(EEXNet) != XNetAddress)
              EEPROM.write(EEXNet, XNetAddress);
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
          client.print("<form method=get>IP-Adr.: <input type=number min=10 max=254 name=A value=");
          client.println(ip[0]);
          client.print(">.<input type=number min=0 max=254 name=B value=");
          client.println(ip[1]);
          client.print(">.<input type=number min=0 max=254 name=C value=");
          client.println(ip[2]);
          client.print(">.<input type=number min=0 max=254 name=D value=");
          client.println(ip[3]);
          client.print("><br /> XBus Adr.: <input type=number min=1 max=31 name=XNet value=");
          client.print(XNetAddress);
          client.println("><br /><br />");
          client.println("<input type=submit></form>");
          client.println("</body></html>");
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


//--------------------------------------------------------------------------------------------
void Ethreceive() {
  int packetSize = Udp.parsePacket();
  if(packetSize > 0) {
    addIPToSlot(Udp.remoteIP()[0], Udp.remoteIP()[1], Udp.remoteIP()[2], Udp.remoteIP()[3]);
    Udp.read(packetBuffer,UDP_TX_PACKET_MAX_SIZE);  // read the packet into packetBufffer
    // send a reply, to the IP address and port that sent us the packet we received
    int header = (packetBuffer[3]<<8) + packetBuffer[2];
//    int datalen = (packetBuffer[1]<<8) + packetBuffer[0];
    byte data[16]; 
    boolean ok = false;
    switch (header) {
      case 0x10:
        data[0] = 0xF5;  //Seriennummer 32 Bit (little endian)
        data[1] = 0x0A;
        data[2] = 0x00; 
        data[3] = 0x00;
        EthSend (0x08, 0x10, data, false, false);
        break; 
      case 0x12:
//Debug.println("Z21-Einstellungen"); 
        break;   
      case 0x1A:
         //Debug.println("LAN_GET_HWINFO"); 
        data[0] = 0x01;  //HwType 32 Bit
        data[1] = 0x02;
        data[2] = 0x00; 
        data[3] = 0x01;
        data[4] = 0x20;  //FW Version 32 Bit
        data[5] = 0x01;
        data[6] = 0x00; 
        data[7] = 0x00;
        EthSend (0x0C, 0x1A, data, false, false);
        break;  
      case 0x30:
         //Debug.println("LAN_LOGOFF"); 
        //Antwort von Z21: keine
        break; 
      case (0x40):
        switch (packetBuffer[4]) { //X-Header
          case 0x21: 
            switch (packetBuffer[5]) {  //DB0
              case 0x21:
                 //Debug.println("LAN_X_GET_VERSION"); 
                data[0] = 0x63;
                data[1] = 0x21;
                data[2] = XBusVer;   //X-Bus Version
                data[3] = 0x12;  //ID der Zentrale
                EthSend (0x09, 0x40, data, true, false);
                break;
              case 0x24:
                 //Debug.println("LAN_X_GET_STATUS"); 
                data[0] = 0x62;
                data[1] = 0x22;
                data[2] = XpressNet.getPower();
                EthSend (0x08, 0x40, data, true, false);
                break;
              case 0x80:
                //Debug.println("LAN_X_SET_TRACK_POWER_OFF"); 
                ok = XpressNet.setPower(csTrackVoltageOff);
                if (ok == false) {
                  // //Debug.println("Power Send FEHLER"); 
                }
                break;
              case 0x81:
                //Debug.println("LAN_X_SET_TRACK_POWER_ON");
                XpressNet.setPower(csNormal);
                if (ok == false) {
                  // //Debug.println("Power Send FEHLER"); 
                }
                break;  
            }
            break;
          case 0x23:
            if (packetBuffer[5] == 0x11) {  //DB0
               //Debug.println("LAN_X_CV_READ"); 
              byte CV_MSB = packetBuffer[6];
              byte CV_LSB = packetBuffer[7];
              XpressNet.readCVMode(CV_LSB+1);
            }
            break;             
          case 0x24:
            if (packetBuffer[5] == 0x12) {  //DB0
               //Debug.println("LAN_X_CV_WRITE"); 
              byte CV_MSB = packetBuffer[6];
              byte CV_LSB = packetBuffer[7];
              byte value = packetBuffer[8]; 
              XpressNet.writeCVMode(CV_LSB+1, value);
            }
            break;             
          case 0x43:
            // //Debug.println("LAN_X_GET_TURNOUT_INFO"); 
            XpressNet.getTrntInfo(packetBuffer[5], packetBuffer[6]);
            break;             
          case 0x53:
            // //Debug.println("LAN_X_SET_TURNOUT"); 
            XpressNet.setTrntPos(packetBuffer[5], packetBuffer[6], packetBuffer[7] & 0x0F);
            break;  
          case 0x80:
             //Debug.println("LAN_X_SET_STOP"); 
            XpressNet.setPower(csEmergencyStop);
            if (ok == false) {
              // //Debug.println("Power Send FEHLER");             
            }
            break;  
          case 0xE3:
            if (packetBuffer[5] == 0xF0) {  //DB0
               //Debug.print("LAN_X_GET_LOCO_INFO: ");
              //Antwort: LAN_X_LOCO_INFO  Adr_MSB - Adr_LSB
               //Debug.println(word(packetBuffer[6], packetBuffer[7]));  //mit F1-F12
              XpressNet.getLocoInfo(packetBuffer[6]& 0x3F, packetBuffer[7]);
              XpressNet.getLocoFunc(packetBuffer[6] & 0x3F, packetBuffer[7]);  //F13 bis F28
            }
            break;  
          case 0xE4:
             //Debug.println("Lok-Adresse");  
            if (packetBuffer[5] == 0xF8) {  //DB0
              //LAN_X_SET_LOCO_FUNCTION  Adr_MSB        Adr_LSB            Type (EIN/AUS/UM)      Funktion
              XpressNet.setLocoFunc(packetBuffer[6] & 0x3F, packetBuffer[7], packetBuffer[8] >> 5, packetBuffer[8] & B00011111); 
            }
            else {
              //LAN_X_SET_LOCO_DRIVE            Adr_MSB          Adr_LSB      DB0          Dir+Speed
              XpressNet.setLocoDrive(packetBuffer[6] & 0x3F, packetBuffer[7], packetBuffer[5] & B11, packetBuffer[8]);       
            }
            break;  
          case 0xE6:
            if (packetBuffer[5] == 0x30) {  //DB0
              byte Option = packetBuffer[8] & B11111100;  //Option DB3
              byte Adr_MSB = packetBuffer[6] & 0x3F;  //DB1
              byte Adr_LSB = packetBuffer[7];    //DB2
              int CVAdr = packetBuffer[9] | ((packetBuffer[8] & B11) << 7);
              if (Option == 0xEC) {
                 //Debug.println("LAN_X_CV_POM_WRITE_BYTE"); 
                byte value = packetBuffer[10];  //DB5
              }
              if (Option == 0xE8) {
                 //Debug.println("LAN_X_CV_POM_WRITE_BIT"); 
                //Nicht von der APP Unterstützt
              }
            }
            break;  
          case 0xF1:
             //Debug.println("LAN_X_GET_FIRMWARE_VERSION"); 
            data[0] = 0xf3;
            data[1] = 0x0a;
            data[2] = 0x01;   //V_MSB
            data[3] = 0x23;  //V_LSB
            EthSend (0x09, 0x40, data, true, false);
            break;     
        }
        break; 
      case (0x50):
         //Debug.print("LAN_SET_BROADCASTFLAGS: "); 
        // //Debug.print(packetBuffer[4], BIN);  // 1=BC Power, Loco INFO, Trnt INFO; B100=BC Sytemstate Datachanged
        break;
      case (0x51):
         //Debug.println("LAN_GET_BROADCASTFLAGS"); 
        break;
      case (0x60):
         //Debug.println("LAN_GET_LOCOMODE"); 
        break;
      case (0x61):
         //Debug.println("LAN_SET_LOCOMODE"); 
        break;
      case (0x70):
         //Debug.println("LAN_GET_TURNOUTMODE"); 
        break;
      case (0x71):
         //Debug.println("LAN_SET_TURNOUTMODE"); 
        break;
      case (0x81):
         //Debug.println("LAN_RMBUS_GETDATA"); 
        break;
      case (0x82):
         //Debug.println("LAN_RMBUS_PROGRAMMODULE"); 
        break;
      case (0x85):
         //Debug.println("LAN_SYSTEMSTATE_GETDATA");  //LAN_SYSTEMSTATE_DATACHANGED
        data[0] = 0x00;  //MainCurrent mA
        data[1] = 0x00;  //MainCurrent mA
        data[2] = 0x00;  //ProgCurrent mA
        data[3] = 0x00;  //ProgCurrent mA        
        data[4] = 0x00;  //FilteredMainCurrent
        data[5] = 0x00;  //FilteredMainCurrent
        data[6] = 0x00;  //Temperature
        data[7] = 0x20;  //Temperature
        data[8] = 0x0F;  //SupplyVoltage
        data[9] = 0x00;  //SupplyVoltage
        data[10] = 0x00;  //VCCVoltage
        data[11] = 0x03;  //VCCVoltage
        data[12] = XpressNet.getPower();  //CentralState
        data[13] = 0x00;  //CentralStateEx
        data[14] = 0x00;  //reserved
        data[15] = 0x00;  //reserved
        EthSend (0x14, 0x84, data, false, false);
        break;
      case (0x89):
         //Debug.println("LAN_RAILCOM_GETDATA"); 
        break;
      case (0xA2):
         //Debug.println("LAN_LOCONET_FROM_LAN"); 
        break;
      case (0xA3):
         //Debug.println("LAN_LOCONET_DISPATCH_ADDR"); 
        break;
      default:
         //Debug.println("LAN_X_UNKNOWN_COMMAND"); 
        data[0] = 0x61;
        data[1] = 0x82;
        EthSend (0x07, 0x40, data, true, false);
    }
  }
}

//--------------------------------------------------------------------------------------------
void EthSend (unsigned int DataLen, unsigned int Header, byte *dataString, boolean withXOR, byte BC) {
  if (BC != 0x00) {
    IPAddress IPout = Udp.remoteIP();
    for (int i = 0; i < maxIP; i++) {
      if (ActIP[i].time > 0 && ActIP[i].BCFlag >= BC) {    //Noch aktiv?
        IPout[0] = ActIP[i].ip0;
        IPout[1] = ActIP[i].ip1;
        IPout[2] = ActIP[i].ip2;
        IPout[3] = ActIP[i].ip3;
        Udp.beginPacket(IPout, Udp.remotePort());    //Broadcast
        Ethwrite (DataLen, Header, dataString, withXOR);
        Udp.endPacket();
      }
    }
  }
  else {
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());    //Broadcast
    Ethwrite (DataLen, Header, dataString, withXOR);
    Udp.endPacket();
  }
}

//--------------------------------------------------------------------------------------------
//Senden von Lokdaten via Ethernet
void Ethwrite (unsigned int DataLen, unsigned int Header, byte *dataString, boolean withXOR) {
  Udp.write(DataLen & 0xFF);
  Udp.write(DataLen >> 8);
  Udp.write(Header & 0xFF);
  Udp.write(Header >> 8);

  unsigned char XOR = 0;
  byte ldata = DataLen-5;  //Ohne Length und Header und XOR
  if (!withXOR)    //XOR vorhanden?
    ldata++;
  for (int i = 0; i < (ldata); i++) {
    XOR = XOR ^ *dataString;
    Udp.write(*dataString);
    dataString++;
  }
  if (withXOR)
    Udp.write(XOR);
}

//--------------------------------------------------------------------------------------------
void notifyXNetPower (uint8_t State)
{
    //Debug.print("Power: 0x"); 
    //Debug.println(State, HEX); 
  byte data[] = {0x61, 0x00};
  switch (State) {
    case csNormal: data[1] = 0x01;
    break;
    case csTrackVoltageOff: data[1] = 0x00;
    break;
    case csServiceMode: data[1] = 0x02;
    break;
    case csEmergencyStop:
            data[0] = 0x81;
            data[1] = 0x00; 
    break;
  }
  EthSend(0x07, 0x40, data, true, true);
}

//--------------------------------------------------------------------------------------------
void notifyLokFunc(uint8_t Adr_High, uint8_t Adr_Low, uint8_t F2, uint8_t F3 ) {
  // //Debug.print("Loco Fkt: "); 
  // //Debug.print(Adr_Low); 
  // //Debug.print(", Fkt2: "); 
  // //Debug.print(F2, BIN); 
  // //Debug.print("; "); 
  // //Debug.println(F3, BIN); 
  
}

//--------------------------------------------------------------------------------------------
void notifyLokAll(uint8_t Adr_High, uint8_t Adr_Low, boolean Busy, uint8_t Steps, uint8_t Speed, uint8_t Direction, uint8_t F0, uint8_t F1, uint8_t F2, uint8_t F3) {

  byte DB2 = Steps;
  if (DB2 == 3)  //nicht vorhanden!
    DB2 = 4;
  if (Busy) 
    bitWrite(DB2, 3, 1);
  byte DB3 = Speed;
  if (Direction == 1)  
    bitWrite(DB3, 7, 1);
  byte data[9]; 
  data[0] = 0xEF;  //X-HEADER
  data[1] = Adr_High & 0x3F;
  data[2] = Adr_Low;
  data[3] = DB2;
  data[4] = DB3;
  data[5] = F0;    //F0, F4, F3, F2, F1
  data[6] = F1;    //F5 - F12; Funktion F5 ist bit0 (LSB)
  data[7] = F2;  //F13-F20
  data[8] = F3;  //F21-F28
  EthSend (14, 0x40, data, true, true);  //Send Power und Funktions to all active Apps
}

//--------------------------------------------------------------------------------------------
void notifyTrnt(uint8_t Adr_High, uint8_t Adr_Low, uint8_t Pos) {
  // //Debug.print("Weiche: "); 
   // //Debug.print(word(Adr_High, Adr_Low)); 
   // //Debug.print(", Position: "); 
   // //Debug.println(Pos, BIN); 
  //LAN_X_TURNOUT_INFO
  byte data[4];
  data[0] = 0x43;  //HEADER
  data[1] = Adr_High;
  data[2] = Adr_Low;
  data[3] = Pos;
  EthSend (0x09, 0x40, data, true, false);  
}

//--------------------------------------------------------------------------------------------
void notifyCVInfo(uint8_t State ) {
   // //Debug.print("CV Prog STATE: "); 
   // //Debug.println(State); 
  if (State == 0x01 || State == 0x02) {  //Busy or No Data
    //LAN_X_CV_NACK
    byte data[2];
    data[0] = 0x61;  //HEADER
    data[1] = 0x13; //DB0
    EthSend (0x07, 0x40, data, true, false);  
  }
}

//--------------------------------------------------------------------------------------------
void notifyCVResult(uint8_t cvAdr, uint8_t cvData ) {
   // //Debug.print("CV Prog Read: "); 
   // //Debug.print(cvAdr); 
   // //Debug.print(", "); 
   // //Debug.println(cvData); 
  //LAN_X_CV_RESULT
  byte data[5];
  data[0] = 0x64; //HEADER
  data[1] = 0x14;  //DB0
  data[2] = 0x00;  //CVAdr_MSB
  data[3] = cvAdr;  //CVAdr_LSB
  data[4] = cvData;  //Value
  EthSend (0x0A, 0x40, data, true, false);
}
