/*
 * DataHandlerInterface.h
 *
 *  Created on: 14.06.2013
 *      Author: ulrich
 */

#ifndef DATAHANDLERINTERFACE_H_
#define DATAHANDLERINTERFACE_H_

class DataHandlerInterface {
public:
	virtual void handleEvent(unsigned char address, unsigned char data)=0;
	virtual ~DataHandlerInterface(){};
};

#endif /* DATAHANDLERINTERFACE_H_ */
