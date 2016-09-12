/*
*****************************************************************************
  *		z21.cpp - library for Roco Z21 LAN protocoll
  *		Copyright (c) 2013-2016 Philipp Gahtow  All right reserved.
  *
  *
*****************************************************************************
  * IMPORTANT:
  * 
  * 	Please contact Roco Inc. for more details.
*****************************************************************************
*/

// include this library's description file
#include <z21.h>
#include <z21header.h>

// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances

z21Class::z21Class()
{
	// initialize this instance's variables 
    z21IPpreviousMillis = 0;
    Railpower = csTrackVoltageOff;
	clearIPSlots();
}

// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries

//*********************************************************************************************
//Daten ermitteln und Auswerten
void z21Class::receive(uint8_t client, uint8_t *packet) 
{
		addIPToSlot(client, 0);
		// send a reply, to the IP address and port that sent us the packet we received
		int header = (packet[3]<<8) + packet[2];
		byte data[16]; 			//z21 send storage
		
		switch (header) {
		case LAN_GET_SERIAL_NUMBER:
		  #if defined(DEBUG)
		  Debug.println("GET_SERIAL_NUMBER");  
		  #endif
		  data[0] = z21SnLSB;
		  data[1] = z21SnMSB;
		  data[2] = 0x00; 
		  data[3] = 0x00;
		  EthSend(client, 0x08, LAN_GET_SERIAL_NUMBER, data, false, Z21bcNone); //Seriennummer 32 Bit (little endian)
		  break; 
		case LAN_GET_HWINFO:
		  #if defined(DEBUG)
		  Debug.println("GET_HWINFO"); 
		  #endif
		  data[0] = z21HWTypeLSB;  //HwType 32 Bit
		  data[1] = z21HWTypeMSB;
		  data[2] = 0x00; 
		  data[3] = 0x00;
		  data[4] = z21FWVersionLSB;  //FW Version 32 Bit
		  data[5] = z21FWVersionMSB;
		  data[6] = 0x00; 
		  data[7] = 0x00;
		  EthSend (client, 0x0C, LAN_GET_HWINFO, data, false, Z21bcNone);
		  break;  
		case LAN_LOGOFF:
		  #if defined(DEBUG)
		  Debug.println("LOGOFF");
		  #endif
		  clearIPSlot(client);
		  //Antwort von Z21: keine
		  break; 
		  case (LAN_X_Header):
		  switch (packet[4]) { //X-Header
		  case LAN_X_GET_SETTING: 
			switch (packet[5]) {  //DB0
			case 0x21:
			  #if defined(DEBUG)
			  Debug.println("X_GET_VERSION"); 
			  #endif
			  data[0] = LAN_X_GET_VERSION;	//X-Header: 0x63
			  data[1] = 0x21;	//DB0
			  data[2] = 0x30;   //X-Bus Version
			  data[3] = 0x12;  //ID der Zentrale
			  EthSend (client, 0x09, LAN_X_Header, data, true, Z21bcNone);
			  break;
			case 0x24:
			  data[0] = LAN_X_STATUS_CHANGED;	//X-Header: 0x62
			  data[1] = 0x22;			//DB0
			  data[2] = Railpower;		//DB1: Status
			  //Debug.print("X_GET_STATUS "); 
				  //csEmergencyStop  0x01 // Der Nothalt ist eingeschaltet 
				  //csTrackVoltageOff  0x02 // Die Gleisspannung ist abgeschaltet 
				  //csShortCircuit  0x04 // Kurzschluss 
				  //csProgrammingModeActive 0x20 // Der Programmiermodus ist aktiv 
			  EthSend (client, 0x08, LAN_X_Header, data, true, Z21bcNone);
			  break;
			case 0x80:
			  #if defined(DEBUG)
			  Debug.println("X_SET_TRACK_POWER_OFF");
			  #endif
			  if (notifyz21RailPower)
				notifyz21RailPower(csTrackVoltageOff);
			  break;
			case 0x81:
			  #if defined(DEBUG)
			  Debug.println("X_SET_TRACK_POWER_ON");
			  #endif
			  if (notifyz21RailPower)
				notifyz21RailPower(csNormal);
			  break;  
			}
			break;  //ENDE DB0
		  case LAN_X_CV_READ:
			if (packet[5] == 0x11) {  //DB0
			  #if defined(DEBUG)
			  Debug.println("X_CV_READ"); 
			  #endif
			  if (notifyz21CVREAD)
				notifyz21CVREAD(packet[6], packet[7]); //CV_MSB, CV_LSB
			}
			break;             
		  case LAN_X_CV_WRITE: 
			if (packet[5] == 0x12) {  //DB0
			  #if defined(DEBUG)
			  Debug.println("X_CV_WRITE"); 
			  #endif
			  byte value = 0;
			  if (notifyz21CVWRITE)
				value = notifyz21CVWRITE(packet[6], packet[7], packet[8]); //CV_MSB, CV_LSB, value
		   
			  data[0] = 0x64; //X-Header
			  data[1] = 0x14; //DB0
			  data[2] = packet[6];  //CV_MSB;
			  data[3] = packet[7]; //CV_LSB;
			  data[4] = value;
			  EthSend (client, 0x0A, LAN_X_Header, data, true, 0x00);
			}
			break;
		  case LAN_X_CV_POM: 
			if (packet[5] == 0x30) {  //DB0
			  uint8_t Adr = ((packet[6] & 0x3F) << 8) + packet[7];
			  uint8_t CVAdr = ((packet[8] & B11) << 8) + packet[9]; 
			  byte value = packet[10];
			  if ((packet[8] >> 2) == B111011) {
				#if defined(DEBUG)
				Debug.println("LAN_X_CV_POM_WRITE_BYTE"); 
				#endif
				if (notifyz21CVPOMWRITEBYTE)
					notifyz21CVPOMWRITEBYTE (Adr, CVAdr, value);  //set decoder 
			  }
			  else if ((packet[8] >> 2) == B111010 && value == 0) {
				#if defined(DEBUG)
				Debug.println("LAN_X_CV_POM_WRITE_BIT"); 
				#endif
			  }
			  else {
				  #if defined(DEBUG)
				  Debug.println("LAN_X_CV_POM_READ_BIYTE"); 
				  #endif
				  byte value = 0;
				  if (notifyz21CVPOMREADBYTE)
					notifyz21CVPOMREADBYTE (Adr, CVAdr);  //set decoder
			  }
			}
			else if (packet[5] == 0x31) {  //DB0
			  #if defined(DEBUG)
			  Debug.println("LAN_X_CV_POM_ACCESSORY"); 
			  #endif
			}
			break;      
		  case LAN_X_GET_TURNOUT_INFO: {
			#if defined(DEBUG)
			  Debug.print("X_GET_TURNOUT_INFO ");
			#endif
			  if (notifyz21AccessoryInfo) {
				  data[0] = 0x43;  //X-HEADER
				  data[1] = packet[5]; //High
				  data[2] = packet[6]; //Low
				  if (notifyz21AccessoryInfo((packet[5] << 8) + packet[6]) == true)
					  data[3] = 0x02;  //active
				  else data[3] = 0x01;  //inactive
			      EthSend (client, 0x09, LAN_X_Header, data, true, Z21bcAll_s);    //BC new 23.04. !!! (old = 0)
			  }
			  break;
		  }
		  case LAN_X_SET_TURNOUT: {
			#if defined(DEBUG)
			Debug.print("X_SET_TURNOUT Adr.:");
			Debug.print((packet[5] << 8) + packet[6]);
			Debug.print(":");
			Debug.print(bitRead(packet[7], 0));
			Debug.print("-");
			Debug.println(bitRead(packet[7], 3));
			#endif
			//bool TurnOnOff = bitRead(packet[7],3);  //Spule EIN/AUS
			if (notifyz21Accessory) {
				notifyz21Accessory((packet[5] << 8) + packet[6], bitRead(packet[7], 0), bitRead(packet[7], 3));
			}						//	Addresse					Links/Rechts			Spule EIN/AUS
			break;  
		  }
		  case LAN_X_SET_STOP:
			#if defined(DEBUG)
			Debug.println("X_SET_STOP");
			#endif
			if (notifyz21RailPower)
				notifyz21RailPower(csEmergencyStop);
			break;  
		  case LAN_X_GET_LOCO_INFO:
			if (packet[5] == 0xF0) {  //DB0
			  //Debug.print("X_GET_LOCO_INFO: ");
			  //Antwort: LAN_X_LOCO_INFO  Adr_MSB - Adr_LSB
			  if (notifyz21getLocoState)
				notifyz21getLocoState(((packet[6] & 0x3F) << 8) + packet[7], false); 
				//Antwort via "setLocoStateFull"!
			}
			break;  
		  case LAN_X_SET_LOCO:
			if (packet[5] == LAN_X_SET_LOCO_FUNCTION) {  //DB0
			  //LAN_X_SET_LOCO_FUNCTION  Adr_MSB        Adr_LSB            Type (EIN/AUS/UM)      Funktion
			  if (notifyz21LocoFkt)
				notifyz21LocoFkt(word(packet[6] & 0x3F, packet[7]), packet[8] >> 5, packet[8] & B00011111); 
			  //uint16_t Adr, uint8_t type, uint8_t fkt
			}
			else {  //DB0
				  //Debug.print("X_SET_LOCO_DRIVE ");
				  byte steps = 14;
				  if ((packet[5] & 0x03) == 3)
					steps = 128;
				  else if ((packet[5] & 0x03) == 2)
					steps = 28;
				if (notifyz21LocoSpeed)
					notifyz21LocoSpeed(word(packet[6] & 0x3F, packet[7]), packet[8],steps);
			}
			break;  
		  case LAN_X_GET_FIRMWARE_VERSION:
			#if defined(DEBUG)
			Debug.println("X_GET_FIRMWARE_VERSION"); 
			#endif
			data[0] = 0xF3;		//identify Firmware (not change)
			data[1] = 0x0A;		//identify Firmware (not change)
			data[2] = z21FWVersionMSB;   //V_MSB
			data[3] = z21FWVersionLSB;  //V_LSB
			EthSend (client, 0x09, LAN_X_Header, data, true, Z21bcNone);
			break;     
		  }
		  break; 
		  case (LAN_SET_BROADCASTFLAGS): {
			unsigned long bcflag = packet[7];
			bcflag = packet[6] | (bcflag << 8);
			bcflag = packet[5] | (bcflag << 8);
			bcflag = packet[4] | (bcflag << 8);
			addIPToSlot(client, getLocalBcFlag(bcflag));
			//no inside of the protokoll, but good to have:
			if (notifyz21RailPower)
				notifyz21RailPower(Railpower); //Zustand Gleisspannung Antworten
			#if defined(DEBUG)
			  Debug.print("SET_BROADCASTFLAGS: "); 
			  Debug.println(addIPToSlot(client, 0x00), BIN);
			  // 1=BC Power, Loco INFO, Trnt INFO; 2=BC aenderungen der Rueckmelder am R-Bus
			#endif
			break;
		  }
		  case (LAN_GET_BROADCASTFLAGS): {
			unsigned long flag = getz21BcFlag(addIPToSlot(client, 0x00));  
			data[0] = flag;
			data[1] = flag >> 8;
			data[2] = flag >> 16;
			data[3] = flag >> 24;
			EthSend (client, 0x08, LAN_GET_BROADCASTFLAGS, data, false, Z21bcNone); 
			#if defined(DEBUG)
			  Debug.print("GET_BROADCASTFLAGS: ");
			  Debug.println(flag, BIN);
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
			  if (notifyz21S88Data) {
				#if defined(DEBUG)
				  Debug.println("RMBUS_GETDATA");
				#endif
				//ask for group state 'Gruppenindex'
				notifyz21S88Data(packet[4]);	//normal Antwort hier nur an den anfragenden Client! (Antwort geht hier an alle!)
			  }
			  break;
		  case (LAN_RMBUS_PROGRAMMODULE):
		  break;
		  case (LAN_SYSTEMSTATE_GETDATA): {	//System state
			  #if defined(DEBUG)
			  Debug.println("LAN_SYS-State");
			  #endif
			  uint16_t maincurrent = 0;
			  if (notifyz21MainCurrent)
				  maincurrent = notifyz21MainCurrent();
			  data[0] = maincurrent & 0xFF;  //MainCurrent mA
			  data[1] = maincurrent >> 8;  //MainCurrent mA
			  data[2] = 0x00;  //ProgCurrent mA
			  data[3] = 0x00;  //ProgCurrent mA        
			  data[4] = 0x01;  //FilteredMainCurrent
			  data[5] = 0x0F;  //FilteredMainCurrent
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
			  EthSend (client, 0x14, LAN_SYSTEMSTATE_DATACHANGED, data, false, Z21bcNone);
			break;
		  }
		  case (LAN_RAILCOM_GETDATA): {
			  uint16_t Adr = 0;
			  if (notifyz21Railcom)
				  Adr = notifyz21Railcom();	//return global Railcom Adr
			  data[0] = Adr >> 8;	//LocoAddress
			  data[1] = Adr & 0xFF;	//LocoAddress
			  data[2] = 0x00;	//UINT32 ReceiveCounter Empfangszaehler in Z21 
			  data[3] = 0x00;
			  data[4] = 0x00;
			  data[5] = 0x00;
			  data[6] = 0x00;	//UINT32 ErrorCounter Empfangsfehlerzaehler in Z21
			  data[7] = 0x00;
			  data[8] = 0x00;
			  data[9] = 0x00;
			  /*
			  data[10] = 0x00;	//UINT8 Reserved1 experimentell, siehe Anmerkung 
			  data[11] = 0x00;	//UINT8 Reserved2 experimentell, siehe Anmerkung 
			  data[12] = 0x00;	//UINT8 Reserved3 experimentell, siehe Anmerkung 
			  */
			  EthSend (client, 0x0E, LAN_RAILCOM_DATACHANGED, data, false, Z21bcNone);
		  }
		  break;
		  case (LAN_LOCONET_FROM_LAN): {
			#if defined(DEBUG)
			  Debug.println("LOCONET_FROM_LAN"); 
			#endif
			if (notifyz21LNSendPacket) {
				byte LNdata[packet[0] - 0x04];  //n Bytes
				for (byte i = 0; i < (packet[0] - 0x04); i++) 
					LNdata[i] = packet[0x04+i];
				notifyz21LNSendPacket(LNdata, packet[0] - 0x04);  
				//Melden an andere LAN-Client das Meldung auf LocoNet-Bus geschrieben wurde
				EthSend(client, packet[0], LAN_LOCONET_FROM_LAN, packet, false, Z21bcLocoNet_s);  //LAN_LOCONET_FROM_LAN
			}
			break;
		  }
		  case (LAN_LOCONET_DISPATCH_ADDR): {
			if (notifyz21LNdispatch) {
				data[0] = packet[4];
				data[1] = packet[5];
				data[2] = notifyz21LNdispatch(packet[5], packet[4]);	//dispatchSlot
				#if defined(DEBUG)
					Debug.print("LOCONET_DISPATCH_ADDR ");
					Debug.print(word(packet[5], packet[4]));
					Debug.print(",");
					Debug.println(data[2]);
				#endif
				EthSend(client, 0x07, LAN_LOCONET_DISPATCH_ADDR, data, false, Z21bcNone);
			}
			break; }
		  case (LAN_LOCONET_DETECTOR):
			  if (notifyz21LNdetector) {
				#if defined(DEBUG)
					Debug.println("LOCONET_DETECTOR Abfrage");
				#endif
				notifyz21LNdetector(packet[4], word(packet[6], packet[5]));	//Anforderung Typ & Reportadresse
			  }
		  break;
		default:
		  #if defined(DEBUG)
			Debug.print("UNKNOWN_COMMAND 0x"); 
			Debug.println(header, HEX);
		  #endif
		  data[0] = 0x61;
		  data[1] = 0x82;
		  EthSend (client, 0x07, LAN_X_Header, data, true, Z21bcNone);
		}
	//---------------------------------------------------------------------------------------
	//check if IP is still used:
	unsigned long currentMillis = millis();
	if ((currentMillis - z21IPpreviousMillis) > z21IPinterval) {
		z21IPpreviousMillis = currentMillis;   
		for (byte i = 0; i < z21clientMAX; i++) {
			if (ActIP[i].time > 0) {
				ActIP[i].time--;    //Zeit herrunterrechnen
			}
			else {
				clearIP(i); 	//clear IP DATA
				//send MESSAGE clear Client
			}
		} 
	}
}

//--------------------------------------------------------------------------------------------
//Zustand der Gleisversorgung setzten
void z21Class::setPower(byte state) 
{
	byte data[] = { LAN_X_BC_TRACK_POWER, 0x00  };
	Railpower = state;
	switch (state) {
		case csNormal: 
				data[1] = 0x01;
				break;
		case csTrackVoltageOff: 
				data[1] = 0x00;
				break;
		case csServiceMode: 
				data[1] = 0x02;
				break;
		case csShortCircuit: 
				data[1] = 0x08;
				break;
		case csEmergencyStop:
				data[0] = 0x81;
				data[1] = 0x00;    
				break;
	}
	EthSend(0, 0x07, LAN_X_Header, data, true, Z21bcAll_s);
	#if defined(DEBUG)
	Debug.print("set_X_BC_TRACK_POWER ");
	Debug.println(state, HEX);
	#endif
}
  
//--------------------------------------------------------------------------------------------
//Abfrage letzte Meldung ueber Gleispannungszustand
byte z21Class::getPower() 
{
	return Railpower;
}

//--------------------------------------------------------------------------------------------
//return request for POM read byte
void z21Class::setCVPOMBYTE (uint16_t CVAdr, uint8_t value) {
	byte data[5]; 
				data[0] = 0x64; //X-Header
				data[1] = 0x14; //DB0
				data[2] = (CVAdr >> 8) & 0x3F;  //CV_MSB;
				data[3] = CVAdr & 0xFF; //CV_LSB;
				data[4] = value;
				EthSend (0, 0x0A, LAN_X_Header, data, true, 0x00);
}				


//--------------------------------------------------------------------------------------------
//Gibt aktuellen Lokstatus an Anfragenden Zurueck
void z21Class::setLocoStateFull (int Adr, byte steps, byte speed, byte F0, byte F1, byte F2, byte F3, bool bc) 
{
	byte data[9]; 
	data[0] = LAN_X_LOCO_INFO;  //0xEF X-HEADER
	data[1] = (Adr >> 8) & 0x3F;
	data[2] = Adr & 0xFF;
	data[3] = steps & B111;		//steps
	if (data[3] == 3)  //nicht vorhanden!
		data[3] = 4;
	data[4] = speed;	//DSSS SSSS
	data[5] = F0;    //F0, F4, F3, F2, F1
	data[6] = F1;    //F5 - F12; Funktion F5 ist bit0 (LSB)
	data[7] = F2;  //F13-F20
	data[8] = F3;  //F21-F28
	if (bc)  //BC?
		EthSend(0, 14, LAN_X_Header, data, true, Z21bcAll_s | Z21bcNetAll_s);  //Send Power und Funktions to all active Apps 
	else EthSend (0, 14, LAN_X_Header, data, true, Z21bcNone);  //Send Power und Funktions to all active Apps
}


//--------------------------------------------------------------------------------------------
//return state of S88 sensors
void z21Class::setS88Data(byte *data) {	
	EthSend(0, 0x0F, LAN_RMBUS_DATACHANGED, data, false, Z21bcRBus_s); //RMBUS_DATACHANED
}

//--------------------------------------------------------------------------------------------
//return state from LN detector
void z21Class::setLNDetector(byte *data, byte DataLen) {
	EthSend(0, 0x04 + DataLen, LAN_LOCONET_DETECTOR, data, false, Z21bcLocoNetGBM_s);  //LAN_LOCONET_DETECTOR
}

//--------------------------------------------------------------------------------------------
//LN Meldungen weiterleiten
void z21Class::setLNMessage(byte *data, byte DataLen, byte bcType, bool TX) {
	if (TX)   //Send by Z21 or Receive a Packet?
		EthSend(0, 0x04 + DataLen, LAN_LOCONET_Z21_TX, data, false, bcType);  //LAN_LOCONET_Z21_TX
	else EthSend(0, 0x04 + DataLen, LAN_LOCONET_Z21_RX, data, false, bcType);  //LAN_LOCONET_Z21_RX
}

//--------------------------------------------------------------------------------------------
//Return the state of accessory
void z21Class::setTrntInfo(uint16_t Adr, bool State) {
	byte data[4];
	data[0] = LAN_X_TURNOUT_INFO;  //0x43 X-HEADER
	data[1] = Adr >> 8;   //High
	data[2] = Adr & 0xFF; //Low
	data[3] = State + 1;
	//  if (State == true)
	//    data[3] = 2;
	//  else data[3] = 1;  
	EthSend(0, 0x09, LAN_X_Header, data, true, Z21bcAll_s);
}


// Private Methods ///////////////////////////////////////////////////////////////////////////////////////////////////
// Functions only available to other functions in this library *******************************************************

//--------------------------------------------------------------------------------------------
void z21Class::EthSend (byte client, unsigned int DataLen, unsigned int Header, byte *dataString, boolean withXOR, byte BC) {
  byte data[24]; 			//z21 send storage
  byte clientOut = client;
  
  for (byte i = 0; i < z21clientMAX; i++) {
    if ((BC == 0) || (BC == Z21bcAll_s) || ((ActIP[i].time > 0) && ((BC & ActIP[i].BCFlag) > 0))) {    //Boradcast & Noch aktiv

      if (BC != 0) {
		if (BC == Z21bcAll_s)
			clientOut = 0;	//ALL
		else clientOut = ActIP[i].client;
      }
      //--------------------------------------------        
	  
	  data[0] = DataLen & 0xFF;
	  data[1] = DataLen >> 8;
	  data[2] = Header & 0xFF;
	  data[3] = Header >> 8;
	  data[DataLen - 1] = 0;	//XOR

      for (byte i = 0; i < (DataLen-5+!withXOR); i++) { //Ohne Length und Header und XOR
        if (withXOR)
			data[DataLen-1] = data[DataLen-1] ^ *dataString;
        //Udp->write(*dataString);
		data[i+4] = *dataString;
        dataString++;
      }
      //--------------------------------------------
      //Udp->endPacket();
	  if (notifyz21EthSend)
		notifyz21EthSend(clientOut, data); //, DataLen);

	  #if defined (DEBUG)
		  Debug.print("ETX ");
		  Debug.print(clientOut);
		  Debug.print(" BC:");
		  Debug.print(BC & ActIP[i].BCFlag, BIN);
		  Debug.print(" : ");
		  for (byte i = 0; i < data[0]; i++) {
			  Debug.print(data[i], HEX);
			  Debug.print(" ");
		  }
		  Debug.println();
	  #endif

      if (BC == 0 || (BC == Z21bcAll_s && clientOut == 0))  //END when no BC
        return;
    }
  }
}

//--------------------------------------------------------------------------------------------
//Convert local stored flag back into a Z21 Flag
unsigned long z21Class::getz21BcFlag (byte flag) {
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
//Convert Z21 LAN BC flag to local stored flag
byte z21Class::getLocalBcFlag (unsigned long flag) {
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
// delete the stored IP-Address
void z21Class::clearIP (byte pos) {
			ActIP[pos].client = 0;
			ActIP[pos].BCFlag = 0;
			ActIP[pos].time = 0;
}

//--------------------------------------------------------------------------------------------
void z21Class::clearIPSlots() {
  for (int i = 0; i < z21clientMAX; i++) 
    clearIP(i);
}

//--------------------------------------------------------------------------------------------
void z21Class::clearIPSlot(byte client) {
  for (int i = 0; i < z21clientMAX; i++) {
	  if (ActIP[i].client == client) {
		  clearIP(i);
		  return;
	  }
  }
}

//--------------------------------------------------------------------------------------------
byte z21Class::addIPToSlot (byte client, byte BCFlag) {
  byte Slot = z21clientMAX;
  for (byte i = 0; i < z21clientMAX; i++) {
    if (ActIP[i].client == client) {
      ActIP[i].time = z21ActTimeIP;
      if (BCFlag != 0)    //Falls BC Flag uebertragen wurde diesen hinzufuegen!
        ActIP[i].BCFlag = BCFlag;
      return ActIP[i].BCFlag;    //BC Flag 4. Byte Rueckmelden
    }
    else if (ActIP[i].time == 0 && Slot == z21clientMAX)
      Slot = i;
  }
  ActIP[Slot].client = client;
  ActIP[Slot].time = z21ActTimeIP;
  setPower(Railpower);
  return ActIP[Slot].BCFlag;   //BC Flag 4. Byte Rueckmelden
}
