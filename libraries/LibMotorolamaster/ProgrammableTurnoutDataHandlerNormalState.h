/*
 * ProgrammableTurnoutDataHandlerNormalState.h
 *
 *  Created on: 14.06.2013
 *      Author: ulrich
 */

#ifndef PROGRAMMABLETURNOUTDATAHANDLERNORMALSTATE_H_
#define PROGRAMMABLETURNOUTDATAHANDLERNORMALSTATE_H_

#include "ProgrammableTurnoutDataHandlerStateInterface.h"
#include "TurnOutDataHandler.h"

class ProgrammableTurnoutDataHandler;

class ProgrammableTurnoutDataHandlerNormalState :
		public ProgrammableTurnoutDataHandlerStateInterface{

	private:
		TurnOutDataHandler _dataHandler;
		unsigned long _ledOnMillis;
		unsigned char _ledState;

	public:

		ProgrammableTurnoutDataHandler* _myMommy;

		ProgrammableTurnoutDataHandlerNormalState();

		void setAdress(int address);
		void setMode(unsigned char mode);
		void setFirstPin(unsigned char firstPin);

		virtual ProgrammableTurnoutDataHandlerStateInterface*
				handleEvent(unsigned char address, unsigned char data);

		virtual ProgrammableTurnoutDataHandlerStateInterface*
				handleTimeouts();

};

#endif /* PROGRAMMABLETURNOUTDATAHANDLERNORMALSTATE_H_ */
