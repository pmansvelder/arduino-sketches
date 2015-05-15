/*
 * ProgrammableTurnoutDataHandlerProgState.h
 *
 *  Created on: 14.06.2013
 *      Author: ulrich
 */

#ifndef PROGRAMMABLETURNOUTDATAHANDLERPROGSTATE_H_
#define PROGRAMMABLETURNOUTDATAHANDLERPROGSTATE_H_
#include "ProgrammableTurnoutDataHandlerStateInterface.h"

class ProgrammableTurnoutDataHandler;

class ProgrammableTurnoutDataHandlerProgState :
		public ProgrammableTurnoutDataHandlerStateInterface {

private:
		unsigned char state;
		unsigned long lastTimeMillis;

public:
	ProgrammableTurnoutDataHandler* myMommy;

	virtual ProgrammableTurnoutDataHandlerStateInterface*
		handleEvent(unsigned char address, unsigned char data);

	virtual ProgrammableTurnoutDataHandlerStateInterface*
			handleTimeouts();
};

#endif /* PROGRAMMABLETURNOUTDATAHANDLERPROGSTATE_H_ */
