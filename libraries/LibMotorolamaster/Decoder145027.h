/*
 * Decode145027.h
 *
 *  Created on: 24.03.2013
 *      Author: ulrich
 */

#ifndef DECODE145027_H_
#define DECODE145027_H_

#include "DataHandlerInterface.h"

class Decoder145027 {

private:
	DataHandlerInterface* handlers[16];
	int handlerFunctionsAdded;


public:
	Decoder145027();

	/*
	*/
	void addHandler(DataHandlerInterface* handler);

	/*
	 *
	 */
	void removeHandler(DataHandlerInterface* handler);

	/*
	 * Decodes a motorola (Version 1) datagram.
	 *
	 * This routine should be called in a loop-function, whenever
	 * the detector detects a datagram.
	 *
	 * You should feed this a pointer to an array of char that has
	 * length 18, like it is provided by the decoder class. You may
	 * directly feed the output of the decoder, there is no need
	 * for a memcopy. If you however decide to feed it too short
	 * an array, the routine will read beyond the bounds of your
	 * data without a warning or an error message.
	 *
	 * As maerklin does with locomotion and turnout datagrams, the
	 * first 4 trits are considered an address. Addresses seem to
	 * be "least-trit-first" in the datagrams on the bus.
	 *
	 * If decoding succeeds, every handler handling the address will be
	 * informed about the data-part of the datagram.
	 *
	 * The data-part is assumed to contain only trits encoding binary
	 * data, so only (long,long) and (short,short) will be accepted.
	 * The corresponding binary data is stored in an unsigned char,
	 * that is passed on to the handlers (see addHandler) for details.
	 *
	 * If decoding succeeds the function will also return the decoded
	 * address of the datagram for every well-formed datagram (that
	 * means for returning of an address it does not matter whether
	 * or not there is a handler for the address registered)
	 *
	 * If decoding fails no handler will be notified. The function
	 * will return -1 or -2, informing about the error:
	 * -1: There was an error in the address part of the datagram.
	 * -2: There was an error in the data-part of the datagram.
	 */
	int decodeDatagram(unsigned char*datagram);


	virtual ~Decoder145027();

};

#endif /* DECODE145027_H_ */
