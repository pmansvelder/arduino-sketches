#include "DCCPacketScheduler.h"
#include "DCCHardware.h"
#include "DDCHardware_config.h"

/*
 * DCC Waveform Generator v2.4
 * 
 * Hardware requirements:
 *     * A DCC booster with Data and GND wired to pin configured in setup routine (PIN 6).
 *     * A locomotive/switch with a DCC decoder installed (NMRA).
 *
 * Author: Philipp Gahtow digitalmoba@arcor.de
 *		   Don Goodman-Wilson dgoodman@artificial-science.org
 *
 * modified by Philipp Gahtow 
 * Copyright 2015-2016 digitalmoba@arcor.de, http://pgahtow.de
 * - add a store for active loco, so you can request the actual state
 * - add a store for BasicAccessory states
 * - add a repeat queue for Speed and Function packets
 * - add Function support F13-F20 and F21-F28
 * - add POM CV Programming
 * - add BasicAccessory increment 4x (Intellibox - ROCO)
 * - add request for state of Loco funktion F0 - F28
 * - support DCC generation with Timer1 or Timer2
 * - add notify of BasicAccessory even when power is off
 * - change praeambel to 16 Bit for Railcom support
 * - add Railcom hardware support 
 *
 * based on software by Wolfgang Kufer, http://opendcc.de
 *
 * Copyright 2010 Don Goodman-Wilson
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
 *  
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *  
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *  
 */

/// The Pin where the DCC Waveform comes out.
extern uint8_t DCCPin;
extern uint8_t DCCPin2;
/// The currently queued packet to be put on the rails. Default is a reset packet.
extern uint8_t current_packet[6];
/// How many data uint8_ts in the queued packet?
extern volatile uint8_t current_packet_size;
/// How many uint8_ts remain to be put on the rails?
extern volatile uint8_t current_uint8_t_counter;
/// How many bits remain in the current data uint8_t/preamble before changing states?
extern volatile uint8_t current_bit_counter; //init to 16 1's for the preamble

///////////////////////////////////////////////
///////////////////////////////////////////////
///////////////////////////////////////////////
  
DCCPacketScheduler::DCCPacketScheduler(void) : /*default_speed_steps(128),*/ last_packet_address(255), packet_counter(1)
{
  e_stop_queue.setup(E_STOP_QUEUE_SIZE);
//  high_priority_queue.setup(HIGH_PRIORITY_QUEUE_SIZE);
//  low_priority_queue.setup(LOW_PRIORITY_QUEUE_SIZE);
  repeat_queue.setup(REPEAT_QUEUE_SIZE);
//NEW
  periodic_refresh_queue.setup(PERIODIC_REFRESH_QUEUE_SIZE);

  ops_programmming_queue.setup(PROG_QUEUE_SIZE);
}

/*
//for configuration
void DCCPacketScheduler::setDefaultSpeedSteps(uint8_t new_speed_steps)
{
  default_speed_steps = new_speed_steps;
}
*/


void DCCPacketScheduler::setup(uint8_t pin, uint8_t format) //for any post-constructor initialization
{
	setup(pin, 0, format);
	#undef RAILCOM
}

void DCCPacketScheduler::setup(uint8_t pin, uint8_t pin2, uint8_t format) //for any post-constructor initialization
{
	#define RAILCOM
	DCCPin = pin;	//set DCC Waveform pin
	DCCPin2 = pin2;	//set Up DCC Waveform pin2
	setup_DCC_waveform_generator();	//Timer neu configurieren
	DCC_waveform_generation_hasshin();
	setpower(OFF);	//no DCC signal active
	slotFullNext = 0;	//don't override, start with free slots
	TrntFormat = format;	//The way BasicAccessory Messages Addressing works (Intellbox/ROCO/etc)

	//Following RP 9.2.4, begin by putting 20 reset packets and 10 idle packets on the rails.
	//reset packet: address 0x00, data 0x00, XOR 0x00; S 9.2 line 75
	opsDecoderReset();	//send first a Reset Packet

	//idle packet: address 0xFF, data 0x00, XOR 0xFF; S 9.2 line 90
	//Automatically send idle packet until first action!
}


//set the power for the dcc signal
void DCCPacketScheduler::setpower(bool state)
{
	railpower = state;	//save the state of the railpower
	//setup_DCC_waveform_generator();	//Timer neu configurieren
	if (state == OFF) {
		DCC_TMR_CONTROL_REG = 0;     // Stop Timer 
		digitalWrite(DCCPin, LOW);	//DCC output pin inaktiv
		digitalWrite(DCCPin2, HIGH);	//DCC output pin inaktiv
	}
	else setup_DCC_waveform_generator();	//Timer neu configurieren
}

//get the actual state of power
byte DCCPacketScheduler::getpower(void)
{
	return railpower;	
}
/*
//helper functions
void DCCPacketScheduler::repeatPacket(DCCPacket *p)
{
  switch(p->getKind())
  {
    case idle_packet_kind:
    case e_stop_packet_kind: //e_stop packets automatically repeat without having to be put in a special queue
      break;
    case speed_packet_kind: //speed packets go to the periodic_refresh queue
    case function_packet_1_kind: //all other packets go to the repeat_queue
    case function_packet_2_kind: //all other packets go to the repeat_queue
    case function_packet_3_kind: //all other packets go to the repeat_queue
	case function_packet_4_kind: //all other packets go to the repeat_queue
	case function_packet_5_kind: //all other packets go to the repeat_queue
		periodic_refresh_queue.insertPacket(p);
		break;
    case accessory_packet_kind:
    case reset_packet_kind:
    case ops_mode_programming_kind:
    case other_packet_kind:
    default:
      repeat_queue.insertPacket(p);
  }
}
*/

//for enqueueing packets

//setSpeed* functions:
//new_speed contains the speed and direction.
// a value of 0 = estop
// a value of 1/-1 = stop
// a value >1 (or <-1) means go.
// valid non-estop speeds are in the range [1,127] / [-127,-1] with 1 = stop
/*
bool DCCPacketScheduler::setSpeed(uint16_t address, uint8_t new_speed, uint8_t steps)
{
  uint8_t num_steps = steps;
  //steps = 0 means use the default; otherwise use the number of steps specified
  if(steps == 0)
    num_steps = default_speed_steps;
        
  switch(num_steps)
  {
    case 14:
      return(setSpeed14(address, new_speed));
    case 28:
      return(setSpeed28(address, new_speed));
    case 128:
      return(setSpeed128(address, new_speed));
  }
  return false; //invalid number of steps specified.
}
*/
bool DCCPacketScheduler::setSpeed14(uint16_t address, uint8_t speed)
{
	if (address == 0)	//check if Adr is ok?
		return false;

	byte slot = LokStsgetSlot(address);
	bitWrite(LokDataUpdate[slot].f0, 5, bitRead(speed, 7));	//Dir
	LokDataUpdate[slot].speed = speed & 0x7F;	//write into register to SAVE
	if ((LokDataUpdate[slot].adr >> 14) != 0)  //0=>14steps, write speed steps into register
		LokDataUpdate[slot].adr = LokDataUpdate[slot].adr & 0x3FFF;		
  
	uint8_t speed_data_uint8_ts[] = {0x40};

    if (speed == 1) //estop!
		//return eStop(address);//
		speed_data_uint8_ts[0] |= 0x01; //estop
    else if (speed == 0) //regular stop!
		speed_data_uint8_ts[0] |= 0x00; //stop
    else //movement
		speed_data_uint8_ts[0] |= map(speed, 2, 127, 2, 15); //convert from [2-127] to [1-14]
    speed_data_uint8_ts[0] |= (0x20 * bitRead(speed, 7)); //flip bit 3 to indicate direction;

    DCCPacket p(address);
    p.addData(speed_data_uint8_ts,1);

    p.setRepeat(SPEED_REPEAT);
  
    p.setKind(speed_packet_kind);  

    //speed packets get refreshed indefinitely, and so the repeat doesn't need to be set.
    //speed packets go to the high proirity queue
    //return(high_priority_queue.insertPacket(&p));
	if (railpower == ESTOP)	//donot send to rails now!
		return periodic_refresh_queue.insertPacket(&p);
	return repeat_queue.insertPacket(&p);
}

bool DCCPacketScheduler::setSpeed28(uint16_t address, uint8_t speed)
{
	if (address == 0)	//check if Adr is ok?
		return false;

  byte slot = LokStsgetSlot(address);
  bitWrite(LokDataUpdate[slot].f0, 5, bitRead(speed, 7));	//Dir
  LokDataUpdate[slot].speed = speed & 0x7F;		// speed & B01111111;	//write into register to SAVE
  if ((LokDataUpdate[slot].adr >> 14) != B10)	//2=>28steps, write into register
	LokDataUpdate[slot].adr = (LokDataUpdate[slot].adr | 0xC000) | 0x8000; 

  uint8_t speed_data_uint8_ts[] = {0x40};

  if(speed == 1) //estop!
    //return eStop(address);//
	speed_data_uint8_ts[0] |= 0x01; //estop
  else if (speed == 0) //regular stop!
    speed_data_uint8_ts[0] |= 0x00; //stop
  else //movement
  {
    speed_data_uint8_ts[0] |= map(speed, 2, 127, 2, 0X1F); //convert from [2-127] to [2-31]  
    //most least significant bit has to be shufled around
    speed_data_uint8_ts[0] = (speed_data_uint8_ts[0]&0xE0) | ((speed_data_uint8_ts[0]&0x1F) >> 1) | ((speed_data_uint8_ts[0]&0x01) << 4);
  }
  speed_data_uint8_ts[0] |= (0x20 * bitRead(speed, 7)); //flip bit 3 to indicate direction;

  DCCPacket p(address);
  p.addData(speed_data_uint8_ts,1);
  
  p.setRepeat(SPEED_REPEAT);
  
  p.setKind(speed_packet_kind);
    
  //speed packets get refreshed indefinitely, and so the repeat doesn't need to be set.
  //speed packets go to the high proirity queue
  //return(high_priority_queue.insertPacket(&p));
  if (railpower == ESTOP)	//donot send to rails now!
	  return periodic_refresh_queue.insertPacket(&p);
  return repeat_queue.insertPacket(&p);
}

bool DCCPacketScheduler::setSpeed128(uint16_t address, uint8_t speed)
{
	if (address == 0) {
	//	Serial.println("ERROR ADR0");
		return false;
	}
	byte slot = LokStsgetSlot(address);
	bitWrite(LokDataUpdate[slot].f0, 5, bitRead(speed, 7));	//Dir
	LokDataUpdate[slot].speed = speed & 0x7F;	//write into register to SAVE
	if ((LokDataUpdate[slot].adr >> 14) != B11) //3=>128steps, write into register
		LokDataUpdate[slot].adr = LokDataUpdate[slot].adr | 0xC000; 

	uint8_t speed_data_uint8_ts[] = { 0x3F, 0x00 };

//	if (speed == 1) //estop!
//		return eStop(address);//speed_data_uint8_ts[1] |= 0x01; //estop
	//else 
		speed_data_uint8_ts[1] = speed; //no conversion necessary.

	//why do we get things like this?
	// 03 3F 16 15 3F (speed packet addressed to loco 03)
	// 03 3F 11 82 AF  (speed packet addressed to loco 03, speed hex 0x11);
	DCCPacket p(address);
	p.addData(speed_data_uint8_ts, 2);

	p.setRepeat(SPEED_REPEAT);

	p.setKind(speed_packet_kind);

	//speed packets get refreshed indefinitely, and so the repeat doesn't need to be set.
	//speed packets go to the high proirity queue

	//return(high_priority_queue.insertPacket(&p));
	if (railpower == ESTOP)	//donot send to rails now!
		return periodic_refresh_queue.insertPacket(&p);
	return repeat_queue.insertPacket(&p);
}

//--------------------------------------------------------------------------------------------
//Lokfunktion setzten
void DCCPacketScheduler::setLocoFunc(uint16_t address, uint8_t type, uint8_t fkt)
{			//type => 0 = AUS; 1 = EIN; 2 = UM; 3 = error
	bool fktbit = 0;	//neue zu ändernde fkt bit
	if (type == 1)	//ein
		fktbit = 1;
	byte Slot = LokStsgetSlot(address);
	//zu änderndes bit bestimmen und neu setzten:
	if ((fkt >= 0) && (fkt <= 4)) {
		byte func = LokDataUpdate[Slot].f0 & B00011111;	//letztes Zustand der Funktionen 000 F0 F4..F1
		if (type == 2) { //um
			if (fkt == 0)
				fktbit = !(bitRead(func, 4));
			else fktbit = !(bitRead(func, fkt - 1));
		}
		if (fkt == 0)
			bitWrite(func, 4, fktbit);
		else bitWrite(func, fkt - 1, fktbit);
		//Daten senden:
		setFunctions0to4(address, func);	//func = 0 0 0 F0 F4 F3 F2 F1
		//Slot anpassen:
//		if (fkt == 0)
//			bitWrite(LokDataUpdate[Slot].f0, 4, fktbit);
//		else bitWrite(LokDataUpdate[Slot].f0, fkt - 1, fktbit);
	}
	else if ((fkt >= 5) && (fkt <= 8)) {
		byte funcG2 = LokDataUpdate[Slot].f1 & 0x0F;	//letztes Zustand der Funktionen 0000 F8..F5
		if (type == 2) //um
			fktbit = !(bitRead(funcG2, fkt - 5));
		bitWrite(funcG2, fkt - 5, fktbit);
		//Daten senden:
		setFunctions5to8(address, funcG2);	//funcG2 = 0 0 0 0 F8 F7 F6 F5
		//Slot anpassen:
//		bitWrite(LokDataUpdate[Slot].f1, fkt - 5, fktbit);
	}
	else if ((fkt >= 9) && (fkt <= 12)) {
		byte funcG3 = LokDataUpdate[Slot].f1 >> 4;	//letztes Zustand der Funktionen 0000 F12..F9
		if (type == 2) //um
			fktbit = !(bitRead(funcG3, fkt - 9));
		bitWrite(funcG3, fkt - 9, fktbit);
		//Daten senden:
		setFunctions9to12(address, funcG3); 	//funcG3 = 0 0 0 0 F12 F11 F10 F9
		//Slot anpassen:
//		bitWrite(LokDataUpdate[Slot].f1, fkt - 9 + 4, fktbit);
	}
	else if ((fkt >= 13) && (fkt <= 20)) {
		byte funcG4 = LokDataUpdate[Slot].f2;
		if (type == 2) //um
			fktbit = !(bitRead(funcG4, fkt - 13));
		bitWrite(funcG4, fkt - 13, fktbit);
		//Daten senden:
		setFunctions13to20(address, funcG4);	//funcG4 = F20 F19 F18 F17 F16 F15 F14 F13
		//Slot anpassen:
//		bitWrite(LokDataUpdate[Slot].f2, (fkt - 13), fktbit);
	}
	else if ((fkt >= 21) && (fkt <= 28)) {
		byte funcG5 = LokDataUpdate[Slot].f3;
		if (type == 2) //um
			fktbit = !(bitRead(funcG5, fkt - 21));
		bitWrite(funcG5, fkt - 21, fktbit);
		//Daten senden:
		setFunctions21to28(address, funcG5);	//funcG5 = F28 F27 F26 F25 F24 F23 F22 F21
		//Slot anpassen:
//		bitWrite(LokDataUpdate[Slot].f3, (fkt - 21), fktbit);
	}
	getLocoStateFull(address, true);	//Alle aktiven Geräte Senden!
}

bool DCCPacketScheduler::setFunctions0to4(uint16_t address, uint8_t functions)
{
	if (address == 0)	//check if Adr is ok?
		return false;

  DCCPacket p(address);
  uint8_t data[] = { 0x80 };
  
  //Obnoxiously, the headlights (F0, AKA FL) are not controlled
  //by bit 0, but in DCC via bit 4. !
  data[0] |= functions & 0x1F;		//new - normal way of DCC! F0, F4, F3, F2, F1

  p.addData(data,1);
  p.setKind(function_packet_1_kind);
  p.setRepeat(FUNCTION_REPEAT);

  LokDataUpdate[LokStsgetSlot(address)].f0 = (LokDataUpdate[LokStsgetSlot(address)].f0 | B00011111) & (functions | B11100000);	//write into register to SAVE

  //return low_priority_queue.insertPacket(&p);
  return repeat_queue.insertPacket(&p);
}


bool DCCPacketScheduler::setFunctions5to8(uint16_t address, uint8_t functions)
{
	if (address == 0)	//check if Adr is ok?
		return false;

  DCCPacket p(address);
  uint8_t data[] = { 0xB0 };
  data[0] |= functions & 0x0F;
  
  p.addData(data,1);
  p.setKind(function_packet_2_kind);
  p.setRepeat(FUNCTION_REPEAT);

  LokDataUpdate[LokStsgetSlot(address)].f1 = (LokDataUpdate[LokStsgetSlot(address)].f1 | 0x0F) & (functions | 0xF0);	//write into register to SAVE

  //return low_priority_queue.insertPacket(&p);
  return repeat_queue.insertPacket(&p);
}

bool DCCPacketScheduler::setFunctions9to12(uint16_t address, uint8_t functions)
{
	if (address == 0)	//check if Adr is ok?
		return false;

  DCCPacket p(address);
  uint8_t data[] = { 0xA0 };
  
  //least significant four functions (F5--F8)
  data[0] |= functions & 0x0F;
  
  p.addData(data,1);
  p.setKind(function_packet_3_kind);
  p.setRepeat(FUNCTION_REPEAT);

  LokDataUpdate[LokStsgetSlot(address)].f1 = (LokDataUpdate[LokStsgetSlot(address)].f1 | 0xF0) & ((functions << 4) | 0x0F);	//write into register to SAVE

  //return low_priority_queue.insertPacket(&p);
  return repeat_queue.insertPacket(&p);
}

bool DCCPacketScheduler::setFunctions13to20(uint16_t address, uint8_t functions)	//F20 F19 F18 F17 F16 F15 F14 F13
{
	if (address == 0)	//check if Adr is ok?
		return false;

	DCCPacket p(address);
	uint8_t data[] = { B11011110, functions }; //significant functions (F20--F13)
	p.addData(data, 2);
	p.setKind(function_packet_4_kind);
	p.setRepeat(FUNCTION_REPEAT);
	LokDataUpdate[LokStsgetSlot(address)].f2 = functions; //write into register to SAVE
	//return low_priority_queue.insertPacket(&p);
	return repeat_queue.insertPacket(&p);
}

bool DCCPacketScheduler::setFunctions21to28(uint16_t address, uint8_t functions)	//F28 F27 F26 F25 F24 F23 F22 F21
{
	if (address == 0)	//check if Adr is ok?
		return false;

	DCCPacket p(address);
	uint8_t data[] = { B11011111, functions}; //significant functions (F28--F21)
	p.addData(data, 2);
	p.setKind(function_packet_5_kind);
	p.setRepeat(FUNCTION_REPEAT);
	LokDataUpdate[LokStsgetSlot(address)].f3 = functions; //write into register to SAVE
	//return low_priority_queue.insertPacket(&p);
	return repeat_queue.insertPacket(&p);
}

byte DCCPacketScheduler::getFunktion0to4(uint16_t address)	//gibt Funktionszustand - F0 F4 F3 F2 F1 zurück
{
	return LokDataUpdate[LokStsgetSlot(address)].f0 & B00011111;
}

byte DCCPacketScheduler::getFunktion5to8(uint16_t address)	//gibt Funktionszustand - F8 F7 F6 F5 zurück
{
	return LokDataUpdate[LokStsgetSlot(address)].f1 & 0x0F;
}

byte DCCPacketScheduler::getFunktion9to12(uint16_t address)	//gibt Funktionszustand - F12 F11 F10 F9 zurück
{
	return LokDataUpdate[LokStsgetSlot(address)].f1 >> 4;
}

byte DCCPacketScheduler::getFunktion13to20(uint16_t address)	//gibt Funktionszustand F20 - F13 zurück
{
	return LokDataUpdate[LokStsgetSlot(address)].f2;
}

byte DCCPacketScheduler::getFunktion21to28(uint16_t address)	//gibt Funktionszustand F28 - F21 zurück
{
	return LokDataUpdate[LokStsgetSlot(address)].f3;
}

//---------------------------------------------------------------------------------
//Special Function for programming, switch and estop:

bool DCCPacketScheduler::setBasicAccessoryPos(uint16_t address, bool state)
{
	return setBasicAccessoryPos(address, state, true);	//Ausgang aktivieren
}

bool DCCPacketScheduler::setBasicAccessoryPos(uint16_t address, bool state, bool activ)
{
	/*
	Accessory decoder packet format:
	================================
	1111..11 0 1000-0001 0 1111-1011 0 EEEE-EEEE 1
      Preamble | 10AA-AAAA | 1aaa-CDDX | Err.Det.B

      aaaAAAAAA -> 111000001 -> Acc. decoder number 1

	  UINT16 FAdr = (FAdr_MSB << 8) + FAdr_LSB;
	  UINT16 Dcc_Addr = FAdr >> 2	//aaaAAAAAA

	  Beispiel:
	  FAdr=0 ergibt DCC-Addr=0 Port=0;
	  FAdr=3 ergibt DCC-Addr=0 Port=3;
	  FAdr=4 ergibt DCC-Addr=1 Port=0; usw

      C on/off:    1 => on		// Ausgang aktivieren oder deaktivieren
      DD turnout: 01 => 2		// FAdr & 0x03  // Port
      X str/div:   1 => set to diverging  // Weiche nach links oder nach rechts 
		=> X=0 soll dabei Weiche auf Abzweig bzw. Signal auf Halt kennzeichnen.

     => COMMAND: SET TURNOUT NUMBER 2 DIVERGING

	 1111..11 0 1000-0001 0 1111-0011 0 EEEE-EEEE 1
	 => COMMAND: SET TURNOUT NUMBER 6 DIVERGING
	*/
	if (address < 0 || address > 0x7FF)	//check if Adr is ok, (max. 11-bit for Basic Adr)
		return false;

	DCCPacket p((address + TrntFormat) >> 2); //9-bit Address + Change Format Roco / Intellibox
	uint8_t data[] = { ((address + TrntFormat) & 0x03) << 1 };	//0000-CDDX
	if (state == true)	//SET X Weiche nach links oder nach rechts 
		bitWrite(data[0], 0, 1);	//set turn
	if (activ == true )	//SET C Ausgang aktivieren oder deaktivieren 
		bitWrite(data[0], 3, 1);	//set ON

	p.addData(data, 1);
	p.setKind(basic_accessory_packet_kind);
	p.setRepeat(OTHER_REPEAT);

	if (notifyTrnt)
		notifyTrnt(address, state);

	bitWrite(BasicAccessory[address / 8], address % 8, state);	//pro SLOT immer 8 Zustände speichern!

	//return high_priority_queue.insertPacket(&p);
	return e_stop_queue.insertPacket(&p);
}

bool DCCPacketScheduler::getBasicAccessoryInfo(uint16_t address)
{
	if (address == 0)	//check if Adr is ok?
		return false;

	switch (TrntFormat) {
		case IB: address = address + IB; break;
	}

	return bitRead(BasicAccessory[address / 8], address % 8);	//Zustand aus Slot lesen
}

bool DCCPacketScheduler::opsProgDirectCV(uint16_t CV, uint8_t CV_data)
{
	//for CV#1 is the Adress 0
	//Long-preamble   0  0111CCAA  0  AAAAAAAA  0  DDDDDDDD  0  EEEEEEEE  1 
	//CC=10 Bit Manipulation
	//CC=01 Verify byte
	//CC=11 Write byte 
	DCCPacket p(((CV >> 8) & B11) | B01111100);
	uint8_t data[] = { 0x00 , CV_data };
	data[0] = CV & 0xFF;
	p.addData(data, 2);
	p.setKind(ops_mode_programming_kind);	//always use short Adress Mode!
	p.setRepeat(OPS_MODE_PROGRAMMING_REPEAT);

	railpower = SERVICE;

	opsDecoderReset();	//send first a Reset Packet
	ops_programmming_queue.insertPacket(&p);
	return opsDecoderReset();	//send a Reset while waiting to finish
}

bool DCCPacketScheduler::opsProgramCV(uint16_t address, uint16_t CV, uint8_t CV_data)
{
	//format of packet:
	// {preamble} 0 [ AAAAAAAA ] 0 111011VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (write)
	// {preamble} 0 [ AAAAAAAA ] 0 111001VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (verify/read)
	// {preamble} 0 [ AAAAAAAA ] 0 111010VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (bit manipulation)
	// only concerned with "write" form here!!!

	if (address == 0)	//check if Adr is ok?
		return false;

	DCCPacket p(address);
	uint8_t data[] = { 0x00, 0x00, 0x00 };

	// split the CV address up among data uint8_ts 0 and 1
	data[0] = ((CV >> 8) & B11) | B11101100;
	data[1] = CV & 0xFF;
	data[2] = CV_data;

	p.addData(data, 3);
	p.setKind(ops_mode_programming_kind);
	p.setRepeat(OPS_MODE_PROGRAMMING_REPEAT);

	//return low_priority_queue.insertPacket(&p);	//Standard

	//railpower = SERVICE;

	//opsDecoderReset();	//send first a Reset Packet
	return ops_programmming_queue.insertPacket(&p);
	//return e_stop_queue.insertPacket(&p);
}

bool DCCPacketScheduler::opsPOMreadCV(uint16_t address, uint16_t CV)
{
	//format of packet:
	// {preamble} 0 [ AAAAAAAA ] 0 111001VV 0 VVVVVVVV 0 DDDDDDDD 0 EEEEEEEE 1 (verify/read)

	if (address == 0)	//check if Adr is ok?
		return false;

	DCCPacket p(address);
	uint8_t data[] = { 0x00, 0x00, 0x00 };

	// split the CV address up among data uint8_ts 0 and 1
	data[0] = ((CV >> 8) & B11) | B11100100;
	data[1] = CV & 0xFF;
	data[2] = 0;

	p.addData(data, 3);
	p.setKind(ops_mode_programming_kind);
	p.setRepeat(OPS_MODE_PROGRAMMING_REPEAT);

	return ops_programmming_queue.insertPacket(&p);
	//return e_stop_queue.insertPacket(&p);
}

//broadcast Decoder ResetPacket
bool DCCPacketScheduler::opsDecoderReset(void)
{
	// {preamble} 0 00000000 0	00000000 0 EEEEEEEE	1
	DCCPacket p(0);	//Adr = 0
	uint8_t data[] = { 0x00 };
	p.addData(data, 1);
	p.setKind(ops_mode_programming_kind);
	p.setRepeat(RESET_REPEAT);
	return ops_programmming_queue.insertPacket(&p);
}
    
//broadcast e-stop command
void DCCPacketScheduler::eStop(void)
{
    // 111111111111 0 00000000 0 01DC0001 0 EEEEEEEE 1
	// C = by default contain one additional speed bit
	// D = direction ("1" the locomotive should	move in	the	forward	direction)
    DCCPacket e_stop_packet(0); //address 0
    uint8_t data[] = {0x61}; //01100001
    e_stop_packet.addData(data,1);
    e_stop_packet.setKind(e_stop_packet_kind);
	e_stop_packet.setRepeat(E_STOP_REPEAT);
    e_stop_queue.insertPacket(&e_stop_packet);

	railpower = ESTOP;
	/*
		//now, clear all other queues
	repeat_queue.clear(speed_packet_kind);				//keep Functions F0-F28
	periodic_refresh_queue.clear(speed_packet_kind);		//keep Functions F0-F28

	//now clear all speed settings
	for (byte i = 0; i < SlotMax; i++) {
		if (!LokStsIsEmpty(i)) { 	//belegter Slot - Lok eingetragen
			//LokDataUpdate[i].speed = 0x01;	
			setSpeed128(LokDataUpdate[i].adr, 0x01 | (bitRead(LokDataUpdate[i].f0, 5) << 7)); //write into register to reset all speed values on ("1")
		}
	}
	*/

    return;
}

/*
bool DCCPacketScheduler::eStop(uint16_t address)
{
    // 111111111111 0	0AAAAAAA 0 01001001 0 EEEEEEEE 1
    // or
    // 111111111111 0	0AAAAAAA 0 01000001 0 EEEEEEEE 1
    DCCPacket e_stop_packet(address);
    uint8_t data[] = {0x41}; //01000001
    e_stop_packet.addData(data,1);
//    e_stop_packet.setKind(e_stop_packet_kind);
	e_stop_packet.setRepeat(E_STOP_REPEAT);
//    e_stop_queue.insertPacket(&e_stop_packet);
    //now, clear this packet's address from all other queues
    //high_priority_queue.forget(address);	//no locos
	//low_priority_queue.forget(address);
	//repeat_queue.forget(address);
	//periodic_refresh_queue.forget(address);			
	e_stop_packet.setKind(speed_packet_kind);
	repeat_queue.insertPacket(&e_stop_packet);
}
*/

//to be called periodically within loop()
//checks queues, puts whatever's pending on the rails via global current_packet
void DCCPacketScheduler::update(void) {
	DCC_waveform_generation_hasshin();
	if (!current_uint8_t_counter) //if the ISR needs a packet:
	{
		DCCPacket p;

		if (ops_programmming_queue.notEmpty()) {	//first Check if ops Service Mode (programming)?
			ops_programmming_queue.readPacket(&p);
		}
		else {
			if (railpower == SERVICE) {		//if command station was in ops Service Mode, switch power off!
				railpower = OFF;
				if (notifyPowerOFF)
					notifyPowerOFF();
			}

			if (e_stop_queue.notEmpty() && (packet_counter % ONCE_REFRESH_INTERVAL)) {	//if there's an e_stop packet, send it now!
				e_stop_queue.readPacket(&p); //nothing more to do. e_stop_queue is a repeat_queue, so automatically repeats where necessary.
			}
			else {
				if (repeat_queue.notEmpty()) {	//for each packet to send it fast, before it stay long waiting in periodic!
					repeat_queue.readPacket(&p);
					periodic_refresh_queue.insertPacket(&p);
				}
				else {
					//	if (periodic_refresh_queue.notEmpty()) // && periodic_refresh_queue.notRepeat(last_packet_address))
					if (railpower != ESTOP)
						periodic_refresh_queue.readPacket(&p);
				}
				/*if (p.getKind() == speed_packet_kind && railpower == ESTOP) {
					e_stop_queue.readPacket(&p);
				}*/
			}
		}
		++packet_counter;	//to not repeat only one queue!
		last_packet_address = p.getAddress(); //remember the address to compare with the next packet
		current_packet_size = p.getBitstream(current_packet); //feed to the starting ISR.
		current_uint8_t_counter = current_packet_size;
	}
}

/*
-> old function:
void DCCPacketScheduler::update(void) 
{
  DCC_waveform_generation_hasshin();

  //TODO ADD POM QUEUE?
  if(!current_uint8_t_counter) //if the ISR needs a packet:
  {
    DCCPacket p;
    //Take from e_stop queue first, then high priority queue.
    //every fifth packet will come from low priority queue.
    //every 20th packet will come from periodic refresh queue. (Why 20? because. TODO reasoning)
    //if there's a packet ready, and the counter is not divisible by 5
    //first, we need to know which queues have packets ready, and the state of the this->packet_counter.
    if( !e_stop_queue.isEmpty() ) //if there's an e_stop packet, send it now!
    {
      //e_stop
      e_stop_queue.readPacket(&p); //nothing more to do. e_stop_queue is a repeat_queue, so automatically repeats where necessary.
    }
    else
    {
      bool doHigh = high_priority_queue.notEmpty() && high_priority_queue.notRepeat(last_packet_address);
      bool doLow = low_priority_queue.notEmpty() && low_priority_queue.notRepeat(last_packet_address) &&
                  !((packet_counter % LOW_PRIORITY_INTERVAL) && doHigh);
      bool doRepeat = repeat_queue.notEmpty() && repeat_queue.notRepeat(last_packet_address) &&
                  !((packet_counter % REPEAT_INTERVAL) && (doHigh || doLow));
//NEW:
      bool doRefresh = periodic_refresh_queue.notEmpty() && periodic_refresh_queue.notRepeat(last_packet_address) &&
                  !((packet_counter % PERIODIC_REFRESH_INTERVAL) && (doHigh || doLow || doRepeat));
      //examine queues in order from lowest priority to highest.
      if(doRefresh)
      {
        periodic_refresh_queue.readPacket(&p);
        ++packet_counter;
      }
      else if(doRepeat)
//NEW (END)
      //if(doRepeat)
      {
        //Serial.println("repeat");
        repeat_queue.readPacket(&p);
        ++packet_counter;
      }
      else if(doLow)
      {
        //Serial.println("low");
        low_priority_queue.readPacket(&p);
        ++packet_counter;
      }
      else if(doHigh)
      {
        //Serial.println("high");
        high_priority_queue.readPacket(&p);
        ++packet_counter;
      }
      //if none of these conditions hold, DCCPackets initialize to the idle packet, so that's what'll get sent.
      //++packet_counter; //it's a uint8_t; let it overflow, that's OK.
      //enqueue the packet for repitition, if necessary:
      //Serial.println("idle");
 //repeatPacket(&p);	//bring the packet into the repeat queue
    }
    last_packet_address = p.getAddress(); //remember the address to compare with the next packet
    current_packet_size = p.getBitstream(current_packet); //feed to the starving ISR.
    //output the packet, for checking:
    //if(current_packet[0] != 0xFF) //if not idle
    //{
    //  for(uint8_t i = 0; i < current_packet_size; ++i)
    //  {
    //    Serial.print(current_packet[i],BIN);
    //    Serial.print(" ");
    //  }
    //  Serial.println("");
    //}
    current_uint8_t_counter = current_packet_size;
  }
}
*/

//--------------------------------------------------------------------------------------------
//Gibt aktuellen Lokstatus an Anfragenden zurück
void DCCPacketScheduler::getLocoStateFull(uint16_t adr, bool bc)
{
	byte Slot = LokStsgetSlot(adr);
	byte Speed = LokDataUpdate[Slot].speed;
	bitWrite(Speed, 7, bitRead(LokDataUpdate[Slot].f0, 5));
	if (notifyLokAll)
		notifyLokAll(adr, /*mode*/LokDataUpdate[Slot].adr >> 14, Speed, /*F0*/LokDataUpdate[Slot].f0 & B00011111, 
		LokDataUpdate[Slot].f1, LokDataUpdate[Slot].f2, LokDataUpdate[Slot].f3, bc);
}

//--------------------------------------------------------------------------------------------
//Gibt aktuelle Fahrtrichtung der Angefragen Lok zurück
byte DCCPacketScheduler::getLocoDir(uint16_t adr)
{
	return bitRead(LokDataUpdate[LokStsgetSlot(adr)].f0, 5);
}
//--------------------------------------------------------------------------------------------
//Gibt aktuelle Geschwindigkeit der Angefragen Lok zurück
byte DCCPacketScheduler::getLocoSpeed(uint16_t adr)
{
	return LokDataUpdate[LokStsgetSlot(adr)].speed;
}

//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
byte DCCPacketScheduler::LokStsgetSlot(uint16_t adr)		//gibt Slot für Adresse zurück / erzeugt neuen Slot (0..126)
{
	byte Slot;
	for (Slot = 0; Slot < SlotMax; Slot++) {
		if ((LokDataUpdate[Slot].adr & 0x3FFF) == adr)
			return Slot;	//Lok gefunden, Slot ausgeben
		if ((LokDataUpdate[Slot].adr & 0x3FFF) == 0) {
			//Empty? neuer freier Slot - keine weitern Lok's!
			LokStsSetNew(Slot, adr);	//Eintragen
			return Slot;
		}
	}
	//kein Slot mehr vorhanden!
	//start am Anfang mit dem Überschreiben vorhandender Slots
	Slot = slotFullNext;
	LokStsSetNew(Slot, adr);	//clear Slot!
	slotFullNext++;
	if (slotFullNext >= SlotMax)
		slotFullNext = 0;
	return Slot;
}

//--------------------------------------------------------------------------------------------
void DCCPacketScheduler::LokStsSetNew(byte Slot, uint16_t adr)	//Neue Lok eintragen mit Adresse
{
	LokDataUpdate[Slot].adr = adr + 0xC000;	// c = '3' => 128 Fahrstufen
	LokDataUpdate[Slot].speed = 0x00;
	LokDataUpdate[Slot].f0 = 0x00;
	LokDataUpdate[Slot].f1 = 0x00;
	LokDataUpdate[Slot].f2 = 0x00;
	LokDataUpdate[Slot].f3 = 0x00;
}

/*
// --------------------------------------------------------------------------------------------
bool DCCPacketScheduler::LokStsIsEmpty(byte Slot)	//prüft ob Datenpacket/Slot leer ist?
{
if ((LokDataUpdate[Slot].adr & 0x3FFF) == 0x0000)
return true;
return false;
}

//--------------------------------------------------------------------------------------------
uint16_t DCCPacketScheduler::LokStsgetAdr(byte Slot)			//gibt Lokadresse des Slot zurück, wenn 0x0000 dann keine Lok vorhanden
{
//	if (!LokDataUpdateIsEmpty(Slot))
		return LokDataUpdate[Slot].adr & 0x3FFF;	//Addresse zurückgeben
//	return 0x0000;
}

//--------------------------------------------------------------------------------------------
byte DCCPacketScheduler::getNextSlot(byte Slot)	//gibt nächsten genutzten Slot
{
	byte nextS = Slot;
	for (byte i = 0; i < SlotMax; i++) {
		nextS++;	//nächste Lok
		if (nextS >= SlotMax)
			nextS = 0;	//Beginne von vorne
		if (LokStsIsEmpty(nextS) == false)
			return nextS;
	}
	return nextS;
}

//--------------------------------------------------------------------------------------------
void DCCPacketScheduler::setFree(uint16_t adr)		//Lok aus Slot nehmen
{
	byte Slot = LokStsgetSlot(adr);
	LokDataUpdate[Slot].adr = 0x0000;
	LokDataUpdate[Slot].speed = 0x00;
	LokDataUpdate[Slot].f0 = 0x00;
	LokDataUpdate[Slot].f1 = 0x00;
	LokDataUpdate[Slot].f2 = 0x00;
	LokDataUpdate[Slot].f3 = 0x00;
}
*/