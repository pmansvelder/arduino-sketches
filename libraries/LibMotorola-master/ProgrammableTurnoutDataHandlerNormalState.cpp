/*
 * ProgrammableTurnoutDataHandlerNormalState.cpp
 *
 *  Created on: 14.06.2013
 *      Author: ulrich
 */

#include "ProgrammableTurnoutDataHandlerNormalState.h"
#include "ProgrammableTurnoutDataHandler.h"
#include "TurnOutDataHandler.h"
#include "Arduino.h"

ProgrammableTurnoutDataHandlerStateInterface*
ProgrammableTurnoutDataHandlerNormalState::handleEvent(unsigned char address,
		unsigned char data) {

	if (address == 0) {
		//Do not switch a turnout, go to progstate!
		return &(_myMommy->_progState);
	}

	//Switch led on for 100ms.
	this->_ledOnMillis = millis();
	this->_ledState = 1;
	digitalWrite(13, 1);

	//Handle the event
	this->_dataHandler.handleEvent(address, data);

	//We stay in normal state.
	return this;
}

void ProgrammableTurnoutDataHandlerNormalState::setAdress(int address) {
	this->_dataHandler.setAddress(address);
}

void ProgrammableTurnoutDataHandlerNormalState::setMode(unsigned char mode){
	this->_dataHandler.setMode(mode);
}

ProgrammableTurnoutDataHandlerNormalState::ProgrammableTurnoutDataHandlerNormalState() :
		_ledState(0), _ledOnMillis(0) {}

void ProgrammableTurnoutDataHandlerNormalState::setFirstPin(
		unsigned char firstPin) {
	((TurnOutDataHandler)this->_dataHandler).setFirstPin(firstPin);
}

ProgrammableTurnoutDataHandlerStateInterface* ProgrammableTurnoutDataHandlerNormalState::handleTimeouts() {

	// Switch of turnouts in case they got stuck.
	this->_dataHandler.handleTimeouts();

	// Switch of light
	if ((this->_ledState != 0) && (abs(millis()-this->_ledOnMillis) > 100)) {
		this->_ledState = 0;
		digitalWrite(13, 0);
	}

	return this;
}
