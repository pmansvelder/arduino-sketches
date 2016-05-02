/*
 * Detect145027.cpp
 *
 *  Created on: 24.03.2013
 *      Author: ulrich
 */

#include "Detect145027.h"
#include "Arduino.h"

Detect145027::Detect145027(unsigned int nominalPulseWidth) {
	this->currentReadLocation = 0;
	this->currentWriteLocation = 0;
	this->currentPulseCounter = 0;
	this->leadingEdgeTimeMicros = 0;
	this->lastLeadingEdgeTimeMicros = 0;
	this->shortPulseTheresholdTimeMicros = nominalPulseWidth / 2;
	this->longPulseTheresholdTimeMicros = nominalPulseWidth *1.1;
}

void Detect145027::pickUpLeadingEdge() {
	//Leading Edge detected...

	lastLeadingEdgeTimeMicros = leadingEdgeTimeMicros;
	leadingEdgeTimeMicros = micros();

	if (currentPulseCounter != 0) {
		unsigned long difference = leadingEdgeTimeMicros
				- lastLeadingEdgeTimeMicros;
		if (difference > longPulseTheresholdTimeMicros) {
			currentPulseCounter = 0;
		}
	}
}

void Detect145027::pickUpTrailingEdge() {

	unsigned long difference = micros() - leadingEdgeTimeMicros;

	if (difference < shortPulseTheresholdTimeMicros) {
		pulseBuffer[currentWriteLocation][currentPulseCounter++] = SHORTPULSE;
	} else if (difference < longPulseTheresholdTimeMicros) {
		pulseBuffer[currentWriteLocation][currentPulseCounter++] = LONGPULSE;
	} else {
		currentPulseCounter = 0;
		return;
	}
	if (currentPulseCounter >= pulsesPerDatagramm) {
		currentPulseCounter = 0;
		unsigned int nextWriteLocation = currentWriteLocation + 1;
		if (nextWriteLocation >= bufferLength)
			nextWriteLocation = 0;
		if (nextWriteLocation == currentReadLocation) {
			// Buffer overflow! I decided to drop the datagram read last.

			// Simply do nothing, which results in the datagram last
			// read being overwritten.
		} else {
			currentWriteLocation = nextWriteLocation;
		}
	}

}

unsigned char* Detect145027::getCurrentDatagram() {
	if (currentReadLocation == currentWriteLocation)
		return NULL;
	volatile unsigned char* result = pulseBuffer[currentReadLocation];
	currentReadLocation++;
	if (currentReadLocation >= bufferLength)
		currentReadLocation = 0;
	return (unsigned char*) result;
}

bool Detect145027::available() {
	return currentReadLocation!=currentWriteLocation;
}

Detect145027::~Detect145027() {
	// TODO Auto-generated destructor stub
}
