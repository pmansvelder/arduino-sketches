/*
  z21.h - library for Z21mobile protocoll
  Copyright (c) 2013-2016 Philipp Gahtow  All right reserved.

  Version 1.3

  ROCO Z21 LAN Protocol for Arduino.
  
  Notice:
	- analyse the data and give back the content and a answer

  Grundlage: Z21 LAN Protokoll Spezifikation V1.05 (21.01.2015)

  Aenderungen:
	- 23.09.15 Anpassung LAN_LOCONET_DETECTOR
			   Fehlerbeseitigung bei der LAN Pruefsumme
			   Anpassung LAN_LOCONET_DISPATCH
	- 14.07.16 add S88 Gruppenindex for request
	- 22.08.16 add POM read notify
*/

// include types & constants of Wiring core API
#if defined(WIRING)
 #include <Wiring.h>
#elif ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

//--------------------------------------------------------------
#define z21Port 21105      // local port to listen on

//**************************************************************
//#define Debug Serial	//Port for the Debugging
//#define DEBUG		//Serial Debug

//**************************************************************
//Firmware-Version der Z21:
#define z21FWVersionMSB 0x01
#define z21FWVersionLSB 0x27
//Hardware-Typ der Z21: 0x00000201 // Z21 (Hardware-Variante ab 2013)
#define z21HWTypeMSB 0x02
#define z21HWTypeLSB 0x01
//Seriennummer:
#define z21SnMSB 0x1A
#define z21SnLSB 0xF5
//--------------------------------------------------------------
// certain global XPressnet status indicators
#define csNormal 0x00 // Normal Operation Resumed ist eingeschaltet
#define csEmergencyStop 0x01// Der Nothalt ist eingeschaltet
#define csTrackVoltageOff 0x02 // Die Gleisspannung ist abgeschaltet
#define csShortCircuit 0x04 // Kurzschluss
#define csServiceMode 0x08 // Der Programmiermodus ist aktiv - Service Mode

#define z21ActTimeIP 20    //Aktivhaltung einer IP fuer (sec./2)
#define z21IPinterval 2000   //interval at milliseconds

struct TypeActIP {
  byte client;    // Byte client
  byte BCFlag;  //BoadCastFlag - see Z21type.h
  byte time;  //Zeit
};

// library interface description
class z21Class
{
  // user-accessible "public" interface
  public:
	z21Class(void);	//Constuctor

	void receive(uint8_t client, uint8_t *packet);				//Pruefe auf neue Ethernet Daten
	
	void setPower(byte state);		//Zustand Gleisspannung Melden
	byte getPower();		//Zusand Gleisspannung ausgeben
	
	void setCVPOMBYTE (uint16_t CVAdr, uint8_t value);	//POM write byte return
	
	void setLocoStateFull (int Adr, byte steps, byte speed, byte F0, byte F1, byte F2, byte F3, bool bc);	//send Loco state 
	unsigned long getz21BcFlag (byte flag);	//Convert local stored flag back into a Z21 Flag
	
	void setS88Data(byte *data);	//return state of S88 sensors

	void setLNDetector(byte *data, byte DataLen);	//return state from LN detector
	void setLNMessage(byte *data, byte DataLen, byte bcType, bool TX);	//return LN Message

	void setTrntInfo(uint16_t Adr, bool State); //Return the state of accessory
	
  // library-accessible "private" interface
  private:

		//Variables:
	byte Railpower;				//state of the railpower
	long z21IPpreviousMillis;        // will store last time of IP decount updated  
	TypeActIP ActIP[z21clientMAX];    //Speicherarray fuer IPs
	
		//Functions:
	void EthSend (byte client, unsigned int DataLen, unsigned int Header, byte *dataString, boolean withXOR, byte BC);
	byte getLocalBcFlag (unsigned long flag);  //Convert Z21 LAN BC flag to local stored flag
	void clearIP (byte pos);		//delete the stored client
	void clearIPSlots();			//delete all stored clients
	void clearIPSlot(byte client);	//delete a client
	byte addIPToSlot (byte client, byte BCFlag);	

};

#if defined (__cplusplus)
	extern "C" {
#endif

	extern uint16_t notifyz21MainCurrent() __attribute__((weak));
	
	extern void notifyz21EthSend(uint8_t client, uint8_t *data) __attribute__((weak));

	extern void notifyz21LNdetector(uint8_t typ, uint16_t Adr) __attribute__((weak));
	extern uint8_t notifyz21LNdispatch(uint8_t Adr2, uint8_t Adr) __attribute__((weak));
	extern void notifyz21LNSendPacket(uint8_t *data, uint8_t length) __attribute__((weak));
	extern void notifyz21RailPower(uint8_t State ) __attribute__((weak));
	
	extern void notifyz21CVREAD(uint8_t cvAdrMSB, uint8_t cvAdrLSB) __attribute__((weak));
	extern uint8_t notifyz21CVWRITE(uint8_t cvAdrMSB, uint8_t cvAdrLSB, uint8_t value) __attribute__((weak));
	extern void notifyz21CVPOMWRITEBYTE(uint16_t Adr, uint16_t cvAdr, uint8_t value) __attribute__((weak));
	extern void notifyz21CVPOMREADBYTE (uint16_t Adr, uint16_t cvAdr) __attribute__((weak));
	
	extern uint8_t notifyz21AccessoryInfo(uint16_t Adr) __attribute__((weak));
	extern void notifyz21Accessory(uint16_t Adr, bool state, bool active) __attribute__((weak));
	extern void notifyz21getLocoState(uint16_t Adr, bool bc) __attribute__((weak));
	extern void notifyz21LocoFkt(uint16_t Adr, uint8_t type, uint8_t fkt) __attribute__((weak));
	extern void notifyz21LocoSpeed(uint16_t Adr, uint8_t speed, uint8_t steps) __attribute__((weak));
	
	extern void notifyz21S88Data(uint8_t gIndex) __attribute__((weak));	//return last state S88 Data for the Client!
	
	extern uint16_t notifyz21Railcom() __attribute__((weak));	//return global Railcom Adr

#if defined (__cplusplus)
}
#endif


