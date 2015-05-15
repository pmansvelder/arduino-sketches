#include <ProgrammableTurnoutDataHandler.h>
#include <ProgrammableTurnoutDataHandlerStateInterface.h>
#include <Decoder145027.h>
#include <TurnOutDataHandler.h>
#include <ProgrammableTurnoutDataHandlerProgState.h>
#include <ProgrammableTurnoutDataHandlerNormalState.h>
#include <DataHandlerInterface.h>
#include <Detect145027.h>


/*
 * 2013-03-31: Ulrich Schwenk
 * You may use this software under the terms of LGPLv3 or any newer version of the LGPL.
 */


#include <avr/io.h>
#include <math.h>

//Globally create a detector.
Detect145027 detector(Detect145027::TURNOUT_NOMINAL_PULSE_WIDTH_MICROS);
Decoder145027 decoder;

void handler() {

	//Do read the pin that corresponds to the interrupt as fast as possible.
	//This is machine-dependent!
	int fastReadResult = PIND & (1 << PD2);

	//Call the appropriate Method on the detector.
	if (fastReadResult != 0) {
		//state is up, must have been leading edge..
		detector.pickUpLeadingEdge();
	} else {
		//must have been trailing edge then...
		detector.pickUpTrailingEdge();
	}
}

TurnOutDataHandler dataHandler;

//The setup function is called once at startup of the sketch
void setup() {
	Serial.begin(115200);
	Serial.println("Initializing!");

	pinMode(13, OUTPUT);
	pinMode(2, INPUT);

	attachInterrupt(0, &handler, CHANGE);
	//Do not use two interrupt handlers on the same pin!
	//(RISING, FALLING)... That didn't work for me.
	//Use CHANGE instead and do a fast read in the interrupt handler
	//to tell leading from trailing edges!

	for (int i = 3; i < 11; i++) {
		pinMode(i, OUTPUT);
	}
        dataHandler.setAddress(79);
        dataHandler.setMode(TurnOutDataHandler::MODESIGNAL);
	decoder.addHandler(&dataHandler);

}

// The loop function is called in an endless loop
void loop() {
	if (detector.available()) {
		unsigned char* datagram = detector.getCurrentDatagram();


		Serial.print("Received Data:");
		for (int i = 0; i < 18; i++) {
			Serial.print((int) datagram[i]);
		}
		Serial.println();

		decoder.decodeDatagram(datagram);
	}

	if (Serial.available()) {
		Serial.read();
		Serial.println("I'm afraid I can't do that, Dave.");
	}

	dataHandler.handleTimeouts();

}
