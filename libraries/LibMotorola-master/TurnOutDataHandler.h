/*
 * TurnOutDataHandler.h
 *
 *  Created on: 14.06.2013
 *      Author: ulrich
 */

#ifndef TURNOUTDATAHANDLER_H_
#define TURNOUTDATAHANDLER_H_

#include "DataHandlerInterface.h"

class TurnOutDataHandler: public DataHandlerInterface {
	private:
		unsigned long _isOnSince[8];
		unsigned char _isOn[8];
		unsigned char _address;
		unsigned char _mode;
		unsigned char _firstPin;

	public:
		static const unsigned char MODETURNOUT=0;
		static const unsigned char MODESIGNAL=1;

		TurnOutDataHandler();

		void setAddress(unsigned char address);
		void setMode(unsigned char mode);
		void setFirstPin(unsigned char firstPin);



		virtual void handleEvent(unsigned char address, unsigned char data);
		virtual void handleTimeouts();
};


#endif /* TURNOUTDATAHANDLER_H_ */
