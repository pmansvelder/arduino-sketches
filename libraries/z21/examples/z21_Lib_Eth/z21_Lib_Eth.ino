/*
  Z21mobile for Arduino
  
  Showing the funktion of the Z21 Library
  connected to a W5100 Ethernet
  
*/

#define DEBUG //To see information from Z21 LAN Protokoll on Serial

#include <z21.h> 
z21Class z21;

//----------------------------------------------------------------------------
#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

//---------------------------------------------------------------
byte mac[] = { 0xFE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
// The IP address will be dependent on your local network:
IPAddress ip(192, 168, 188, 111);

#define Z21_UDP_TX_MAX_SIZE 15  //--> POM DATA has 12 Byte!
unsigned char packetBuffer[Z21_UDP_TX_MAX_SIZE]; //buffer to hold incoming packet,

#define maxIP 20  //Größe des IP-Speicher
typedef struct		//Rückmeldung des Status der Programmierung
{
  byte IP0;
  byte IP1;
  byte IP2;
  byte IP3;
} listofIP;
listofIP mem[maxIP];
byte storedIP = 0;  //speicher für IPs

/*********************************************************************************************/
// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  Serial.begin(115200);
  Serial.println("Z21mobile");
  
  // start the Ethernet and UDP:
  Ethernet.begin(mac,ip);  //IP and MAC Festlegung
  Udp.begin(z21Port);  //UDP Z21 Port
  
  z21.setPower(csNormal);
}

/*********************************************************************************************/
// the loop routine runs over and over again forever:
void loop() {
  
  //--------------------------------------------------------------------------------------------
  if(Udp.parsePacket() > 0) {  //packetSize
    Udp.read(packetBuffer,Z21_UDP_TX_MAX_SIZE);  // read the packet into packetBufffer
    IPAddress remote = Udp.remoteIP();
    z21.receive(addIP(remote[0], remote[1], remote[2], remote[3]), packetBuffer);
  }
  
   //-------------------------------------------------------------------------------------------- 
   //unsigned long getz21BcFlag (byte flag); 
   //byte getPower();  //Zusand Gleisspannung ausgeben
   //void setS88Data(byte *data); //return state of S88 sensors
   //void setLNDetector(byte *data); //return state from LN detector
   //void setLNMessage(byte *data, byte DataLen, byte bcType, bool TX);  //return LN Message
   //void setTrntInfo(uint16_t Adr, bool State); //Return the state of accessory
}

//--------------------------------------------------------------------------------------------
byte addIP (byte ip0, byte ip1, byte ip2, byte ip3) {
  //suche ob IP schon vorhanden?
  for (byte i = 0; i < storedIP; i++) {
    if (mem[i].IP0 == ip0 && mem[i].IP1 == ip1 && mem[i].IP2 == ip2 && mem[i].IP3 == ip3)
      return i+1;
  }
  if (storedIP >= maxIP)
    return 0;
  mem[storedIP].IP0 = ip0;
  mem[storedIP].IP1 = ip1;
  mem[storedIP].IP2 = ip2;
  mem[storedIP].IP3 = ip3;
  storedIP++;
  return storedIP;  
}

//--------------------------------------------------------------------------------------------
void notifyz21RailPower(uint8_t State)
{
  Serial.print("Power: ");
  Serial.println(State, HEX);
}

//--------------------------------------------------------------------------------------------
void notifyz21EthSend(uint8_t client, uint8_t *data) 
{
  if (client == 0) { //all stored 
    for (byte i = 0; i < storedIP; i++) {
      IPAddress ip(mem[i].IP0, mem[i].IP1, mem[i].IP2, mem[i].IP3);
      Udp.beginPacket(ip, Udp.remotePort());    //Broadcast
      Udp.write(data, data[0]);
      Udp.endPacket();
    }
  }
  else {
    IPAddress ip(mem[client-1].IP0, mem[client-1].IP1, mem[client-1].IP2, mem[client-1].IP3);
    Udp.beginPacket(ip, Udp.remotePort());    //no Broadcast
    Udp.write(data, data[0]);
    Udp.endPacket();
  }
}

//--------------------------------------------------------------------------------------------
void notifyz21S88Data()
{
  //z21.setS88Data (datasend);  //Send back state of S88 Feedback
}

//--------------------------------------------------------------------------------------------
void notifyz21getLocoState(uint16_t Adr, bool bc)
{
  //void setLocoStateFull (int Adr, byte steps, byte speed, byte F0, byte F1, byte F2, byte F3, bool bc);
}

void notifyz21LocoFkt(uint16_t Adr, uint8_t type, uint8_t fkt)
{

}

//--------------------------------------------------------------------------------------------
void notifyz21LocoSpeed(uint16_t Adr, uint8_t speed, uint8_t steps)
{

}

//--------------------------------------------------------------------------------------------
void notifyz21Accessory(uint16_t Adr, bool state, bool active)
{

}

//--------------------------------------------------------------------------------------------
uint8_t notifyz21AccessoryInfo(uint16_t Adr)
//return state of the Address (left/right = true/false)
{
  return false;
}

//--------------------------------------------------------------------------------------------
uint8_t notifyz21LNdispatch(uint8_t Adr2, uint8_t Adr)
//return the Slot that was dispatched, 0xFF at error!
{
  return 0xFF;
}

//--------------------------------------------------------------------------------------------
void notifyz21LNSendPacket(uint8_t *data, uint8_t length)
{

}

//--------------------------------------------------------------------------------------------
void notifyz21CVREAD(uint8_t cvAdrMSB, uint8_t cvAdrLSB)
{

}

//--------------------------------------------------------------------------------------------
void notifyz21CVWRITE(uint8_t cvAdrMSB, uint8_t cvAdrLSB, uint8_t value)
{

}

//--------------------------------------------------------------------------------------------
void notifyz21CVPOMWRITEBYTE(uint16_t Adr, uint16_t cvAdr, uint8_t value)
{

}

//--------------------------------------------------------------------------------------------
