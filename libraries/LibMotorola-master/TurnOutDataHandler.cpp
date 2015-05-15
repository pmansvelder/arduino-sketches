/*
 * TurnOutDataHandler.cpp
 *
 *  Created on: 14.06.2013
 *      Author: ulrich
 */

#include "TurnOutDataHandler.h"
#include "Arduino.h"

TurnOutDataHandler::TurnOutDataHandler() :
_address(1), _mode(MODETURNOUT), _firstPin(3)
{

}

void TurnOutDataHandler::handleEvent(unsigned char address, unsigned char data) {

	// If it's a foreign address, ignore it.
	if(address!=this->_address) return;

	//least significant bit is always zero.
	//the preceding three bits hold the payload
	//(which tells us the coil to activate/deactivate)
	unsigned char payload = (data & 0x0F) >> 1;

	//The fifth bit gives the enable-state
	unsigned char enable = (data & 0x10) >> 4;

	//Record values needed by turnout-protection.
	this->_isOnSince[payload]= millis();
	this->_isOn[payload] = enable;


	//If in signalmode...
	if(_mode==MODESIGNAL){
		//And as soon as a even coil is concerned (all even coils stand for "red")
		if((payload & 0x01)==0){
			//Activate the corresponding pin.
			//Deactivate the next pin, which is the corresponding "green" pin.
			digitalWrite(_firstPin+payload,1);
			digitalWrite(_firstPin+payload+1,0);
		} else {
			//Activate the corresponding pin.
			//Deactivate the preceding pin, which is the corresponding "red" pin.
			digitalWrite(_firstPin+payload-1,0);
			digitalWrite(_firstPin+payload,1);
		}
	} else {
		digitalWrite(_firstPin + payload, enable);
	}
}

void TurnOutDataHandler::setAddress(unsigned char address) {
	this->_address=address;
}

void TurnOutDataHandler::setMode(unsigned char mode) {
	this->_mode=mode;
	// When going into the new mode, set the lights accordingly.
	for(int i=0;i<8;i++){
		//If in Signalmode, let light up all red lights
		//If in Turnoutmode, make sure no output is stuck.
		digitalWrite(3+i,this->_mode==MODESIGNAL ? (!(i & 0x01)) : 0);

		//In Turnoutmode all lights are "unstuck".
		//In Signalmode it's not used, therefore setting it to zero does not harm anything.
		_isOn[i]=0;
	}
}

void TurnOutDataHandler::setFirstPin(unsigned char firstPin) {
	this->_firstPin=firstPin;
}

void TurnOutDataHandler::handleTimeouts(){

	//If in Signalmode Turnoutprotection is not needed.
	if(_mode==MODESIGNAL) return;

	//Turnout protection. Turn off after 500ms even if there was no switch-off signal.
	for(int i=0;i<8;i++){
		if((_isOn[i]!=0) && (abs( millis()-_isOnSince[i])>500)){
			_isOn[i]=0;
			digitalWrite(_firstPin+i,0);
		}
	}

}
