/*
 * Detect145027.h
 *
 *  Created on: 24.03.2013
 *      Author: ulrich
 */

#ifndef DETECT145027_H_
#define DETECT145027_H_


class Detect145027 {


private:
	/* Gives the bufferlength in diagrams.
	 * 100 datagrams looks like a sensible buffer-size to my eyes
	 */
	static const unsigned int bufferLength=25;

	/* Defines the length of a diagram in pulses. Maerklin-Motorola datagrams
	 * always consist of 9 ternary bits represented by 2 pulses each. So
	 * this is 18 for all maerklin related stuff.
	 */
	static const unsigned int pulsesPerDatagramm=18;



	/* If shortPulseTheresholdTimeMicros microseconds after leading edge
	 * Signal is still high, it is a long pulse!*/
	unsigned int shortPulseTheresholdTimeMicros;

	/* if there is more microseconds between two leading edges than
	 * this value specifies, a pulse will be regarded as to long */
	unsigned int longPulseTheresholdTimeMicros;

	/* the last leading edge happened when micros() was equal to this */
	volatile unsigned long leadingEdgeTimeMicros;

	/* the leading edge befor the last leading edge happened when
	 * micros() was equal to this. Used to detect if a pulse is to long */
	volatile unsigned long lastLeadingEdgeTimeMicros;


	/* Holds the pulses received.
	 * first dimension is for datagrams in the buffer.
	 * second dimension gives the pulses in a datagram.
	 * if you want to adress the m-th pulse of the n-th datagram read
	 * pulseBuffer[n][m]
	 */
	volatile unsigned char pulseBuffer[bufferLength][pulsesPerDatagramm];
	/* Gives the datagram in the buffer that will be read next.
	 * if currentReadLocation equals currentWriteLocation there is no
	 * datagram ready to be read.
	 */
	volatile unsigned int currentReadLocation;

	/*
	 * Gives the location where currently a datagram is received to.
	 */
	volatile unsigned int currentWriteLocation;

	/*
	 * Gives the number of the pulse that is currently written
	 * to currentWriteLocation
	 */
	volatile unsigned int currentPulseCounter;


public:

	static const unsigned char SHORTPULSE=0;
	static const unsigned char LONGPULSE=1;
	/*
	 * value to set nominalPulseWidth to for locomotion-decoding...
	 */
	static const unsigned int LOCOMOTION_NOMINAL_PULSE_WIDTH_MICROS = 208;

	/*
	 * value to set nominalPulseWith to for turnout-decoding
	 */
	static const unsigned int TURNOUT_NOMINAL_PULSE_WIDTH_MICROS = 104;

	void pickUpLeadingEdge();
	void pickUpTrailingEdge();
	unsigned char* getCurrentDatagram();
	bool available();

	/*
	 * Creates the detector for a nominal pulse width.
	 * Nominal Pulse width means the time in microseconds after which the next pulse is to
	 * be received according to the protocol. Maeklin specifies 208 microseconds
	 * per ternary bit for turnout-decoders, 416 for locomotion-decoders. As
	 * each ternary bit is formed by two pulses, this is
	 * 208/2 = 104 for turnout-decoding
	 * 416/2 = 208 for locomotion-decoding
	 * Best use the in-class constants to initialize this for turnout or locomotion-decoding...
	 */
	unsigned int nominalPulseWidth;
	Detect145027(unsigned int nominalPulseWidth);
	virtual ~Detect145027();
};

#endif /* DETECT145027_H_ */
