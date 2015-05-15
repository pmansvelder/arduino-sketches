/*
 * ProgrammableTurnoutDataHandler.h
 *
 *  Created on: 14.06.2013
 *      Author: ulrich
 */

#ifndef PROGRAMMABLETURNOUTDATAHANDLER_H_
#define PROGRAMMABLETURNOUTDATAHANDLER_H_
#include "DataHandlerInterface.h"
#include "ProgrammableTurnoutDataHandlerStateInterface.h"
#include "ProgrammableTurnoutDataHandlerNormalState.h"
#include "ProgrammableTurnoutDataHandlerProgState.h"

class ProgrammableTurnoutDataHandler : public DataHandlerInterface {

private:
	ProgrammableTurnoutDataHandlerStateInterface* _mystate;

public:
	ProgrammableTurnoutDataHandlerNormalState _normalState;
	ProgrammableTurnoutDataHandlerProgState _progState;

	ProgrammableTurnoutDataHandler();
	virtual void setFirstPin(unsigned char pin);
	virtual void handleEvent(unsigned char address, unsigned char data);
	virtual void handleTimeouts();

};

#endif /* PROGRAMMABLETURNOUTDATAHANDLER_H_ */
