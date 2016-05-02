/*
 * ProgrammableTurnoutDataHandlerProgState.cpp
 *
 *  Created on: 14.06.2013
 *      Author: ulrich
 */

#include "ProgrammableTurnoutDataHandlerProgState.h"
#include "ProgrammableTurnoutDataHandlerNormalState.h"
#include "ProgrammableTurnoutDataHandler.h"
#include "Arduino.h"
#include "avr/eeprom.h"

ProgrammableTurnoutDataHandlerStateInterface*
ProgrammableTurnoutDataHandlerProgState::handleEvent(unsigned char address,
		unsigned char data) {
	if (address == 0) {
		return this; //Stay in Progmode
	} else {
		//set new address
		Serial.print("Payload programming is: ");
		unsigned char payload = (data & 0x0F) >> 1;
		Serial.println(payload);
		unsigned char mode = (payload & 0x01) != 0 ?
				TurnOutDataHandler::MODETURNOUT :
				TurnOutDataHandler::MODESIGNAL;

		digitalWrite(13,0);
		eeprom_write_byte((uint8_t*) 0,address);
		eeprom_write_byte((uint8_t*) 1,mode);
		this->myMommy->_normalState.setAdress(address);
		this->myMommy->_normalState.setMode(mode);
		return &(this->myMommy->_normalState);
	}

}

ProgrammableTurnoutDataHandlerStateInterface* ProgrammableTurnoutDataHandlerProgState::handleTimeouts() {
	if (abs(millis() - lastTimeMillis) > 100) {
		lastTimeMillis=millis();
		state = state != 0 ? 0 : 1;
		digitalWrite(13, state);
	}
	return &(this->myMommy->_progState);
}
