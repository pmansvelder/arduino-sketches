/*
 * Decode145027.cpp
 *
 *  Created on: 24.03.2013
 *      Author: ulrich
 */

#include "Decoder145027.h"
#include "Detect145027.h"
#include "Arduino.h"

Decoder145027::Decoder145027() {
	handlerFunctionsAdded = 0;
}


void Decoder145027::addHandler(DataHandlerInterface* handler) {
	this->handlers[this->handlerFunctionsAdded] = handler;
	this->handlerFunctionsAdded++;
}

void Decoder145027::removeHandler(DataHandlerInterface* handler){
	if(handlerFunctionsAdded==0) return;

	//If nothing is found this index is to high for the overwrite-loop to run!
	int index=this->handlerFunctionsAdded;
	//Now find the position for the handler to erase
	for(int i=0;i<this->handlerFunctionsAdded;i++){
		if (this->handlers[i]==handler){
			index = i;
			break;
		}
	}

	//Adapt number of handlers
	handlerFunctionsAdded--;

	//Overwrite array with the handler following in the array
	//starting from the handler to be erased
	for(int i=index;i<handlerFunctionsAdded;i++){
		this->handlers[i]=this->handlers[i+1];
	}
}

int Decoder145027::decodeDatagram(unsigned char* datagram) {

	// Datagram contains the Motorola-Datagram (18 Pulses,
	// that are either LONG or SHORT.
	// Long,Long means 1
	// Short, Short means 0
	// Long, Short means 2
	// and Short, Long is illegal in the Maerklin V1 Protocol.

	// Examine first 4 Trits
	// They contain the address.
	// The Address is encoded "least-significant-trit" first.
	unsigned int address=0;
	unsigned int base=1;
	for (int i = 0; i < 8; i += 2) {
		unsigned int currentResult = 0;
		if (datagram[i + 0] == Detect145027::LONGPULSE) {
			if (datagram[i + 1] == Detect145027::LONGPULSE) {
				currentResult = 1;
			} else if (datagram[i + 1] == Detect145027::SHORTPULSE) {
				currentResult = 2;
			} else {
				return -1; //Invalid coding!
			}
		} else if(datagram[i+0] == Detect145027::SHORTPULSE){
			if(datagram[i+1] == Detect145027::SHORTPULSE){
				currentResult=0;
			} else {
				return -1;
			}
		} else {
			return -1;
		}
		address+=base*currentResult;
		base=base*3;
	}

	// Get Databits.
	unsigned char data=0;
	for(int k=8;k<18;k+=2){
		if(datagram[k+0]==Detect145027::LONGPULSE && datagram[k+1]==Detect145027::LONGPULSE){
			//Data will be shifted 5 times, so set the 6th bit.
			data=data|0x20;
		} else if (datagram[k+0]==Detect145027::SHORTPULSE && datagram[k+1]==Detect145027::SHORTPULSE){
			//NOP
		} else {
			return -2; //Illegal coding!
		}
		data=data>>1;
	}


	// Inform all handlers that match address
	for (int j=0;j<this->handlerFunctionsAdded;j++){
			this->handlers[j]->handleEvent(address,data);
	}

	// Return the address
	return address;

}

Decoder145027::~Decoder145027() {
	// TODO Auto-generated destructor stub
}
