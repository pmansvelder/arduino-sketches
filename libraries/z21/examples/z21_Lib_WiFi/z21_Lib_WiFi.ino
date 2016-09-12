/*
  Z21mobile for Arduino
  
  Showing the funktion of the Z21 Library
  connected to a ESP8266 interface via Serial1
  (Note: The ESP8266 has a specific Z21 firmware in this case!)
  
*/

#define DEBUG //To see information from Z21 LAN Protokoll on Serial

#include <z21.h> 
z21Class z21;

//----------------------------------------------------------------------------
#define Debug Serial   //Debug Port for log Informationen
#define ESP Serial1    // Port from ESP

#define Z21_UDP_TX_MAX_SIZE 15  //--> POM DATA has 12 Byte!
unsigned char packetBuffer[Z21_UDP_TX_MAX_SIZE]; //buffer to hold incoming packet,

byte inDcount = 0;    //Zähler für eingelesene Daten
byte sendFrom = 0xFF;  //client der die Daten gesendet hat

/*********************************************************************************************/
// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  Debug.begin(115200);
  ESP.begin(115200);
    
  z21.setPower(csNormal);
}

/*********************************************************************************************/
// the loop routine runs over and over again forever:
void loop() {
  
  if (ESP.available() > 0) {  //Receive UDP
      packetBuffer[inDcount] = ESP.read();
      if (sendFrom == 0xFF)
        sendFrom = packetBuffer[inDcount];  //client auslesen
      else {
        inDcount++;
        if (packetBuffer[0] <= inDcount) {  //1. Byte gibt länge der Daten an!
          Debug.write(sendFrom);
          Debug.write(packetBuffer, packetBuffer[0]);
          Debug.print("OK");  //Accept
          z21.receive(sendFrom, packetBuffer);  //Auswertung
          inDcount = 0;
          sendFrom = 0xFF;
        }
        if (inDcount >= Z21_UDP_TX_MAX_SIZE) {  //keine valied Data!
          Debug.print("invalied Data");  //Fail
          inDcount = 0;
          sendFrom = 0xFF;
          ESP.flush();
        }
      }
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
void notifyz21RailPower(uint8_t State)
{
  Debug.print("Power: ");
  Debug.println(State, HEX);
}

//--------------------------------------------------------------------------------------------
void notifyz21EthSend(uint8_t client, uint8_t *data) 
{
  ESP.write(client);
  ESP.write(data, data[0]);  
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

