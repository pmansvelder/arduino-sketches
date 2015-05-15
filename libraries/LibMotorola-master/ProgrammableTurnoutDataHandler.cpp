/*
 * ProgrammableTurnoutDataHandler.cpp
 *
 *  Created on: 14.06.2013
 *      Author: ulrich
 */

#include "ProgrammableTurnoutDataHandler.h"
#include "Arduino.h"
#include "avr/eeprom.h"

ProgrammableTurnoutDataHandler::ProgrammableTurnoutDataHandler(){
	this->_progState.myMommy=this;
	this->_normalState._myMommy=this;
	unsigned char result = eeprom_read_byte((uint8_t*)0);
	if(result==255) result = 1;
	this->_normalState.setAdress(result);

	result = eeprom_read_byte((uint8_t*)1);
	if (result>1) result = 0;
	this->_normalState.setMode(result);

	this->_mystate = &(this->_normalState);
}

void ProgrammableTurnoutDataHandler::handleEvent(unsigned char address,
		unsigned char data) {
	ProgrammableTurnoutDataHandlerStateInterface* nextState =
			this->_mystate->handleEvent(address, data);
	this->_mystate = nextState;
}

void ProgrammableTurnoutDataHandler::setFirstPin(unsigned char pin) {
	this->_normalState.setFirstPin(pin);
}

void ProgrammableTurnoutDataHandler::handleTimeouts(){
	ProgrammableTurnoutDataHandlerStateInterface* nextState =
				this->_mystate->handleTimeouts();
	this->_mystate=nextState;
}
