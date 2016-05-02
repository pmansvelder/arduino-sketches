/*
 * ProgrammableTurnoutDataHandlerStateInterface.h
 *
 *  Created on: 14.06.2013
 *      Author: ulrich
 */

#ifndef PROGRAMMABLETURNOUTDATAHANDLERSTATEINTERFACE_H_
#define PROGRAMMABLETURNOUTDATAHANDLERSTATEINTERFACE_H_


class ProgrammableTurnoutDataHandlerStateInterface {

public:
	virtual ProgrammableTurnoutDataHandlerStateInterface*
		handleEvent(unsigned char address, unsigned char data)=0;

	virtual ProgrammableTurnoutDataHandlerStateInterface*
		handleTimeouts()=0;

	virtual ~ProgrammableTurnoutDataHandlerStateInterface(){};

};

#endif /* PROGRAMMABLETURNOUTDATAHANDLERSTATEINTERFACE_H_ */
