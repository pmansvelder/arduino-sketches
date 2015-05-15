/*
 * 2013-03-31: Ulrich Schwenk
 * You may use this software under the terms of LGPLv3 or any newer version of the LGPL.
 */
#include <ProgrammableTurnoutDataHandler.h>
#include <Decoder145027.h>
#include <Detect145027.h>
#include <avr/io.h>
#include <math.h>

// Globally create a detector. It filters märklin-datagrams from the datastream and buffers them.
Detect145027 detector(Detect145027::TURNOUT_NOMINAL_PULSE_WIDTH_MICROS);
// Globally create a decoder. It reads data from the detectors buffer and extracts adresses and payloads.
Decoder145027 decoder;
// Create a Handler that makes sence of the payload and does the heavy lifting.
// This one creates a programmable decoder-unit, that can be used for signals as well as turnouts.
ProgrammableTurnoutDataHandler dataHandler;

// This is the interrupthandler. You have to write that code to tell the detector about the
// leading and trailing edges of the model railway-controller's signal.
// As this routine does a fast-read of the pins it will usually depend on what
// arduino hardware you actually have and what pins you plan to use.
void handler() {

	//Do read the pin that corresponds to the interrupt as fast as possible.
	//This is machine-dependent!
	int fastReadResult = PIND & (1 << PD2);    //Arduino UNO using pin 2, int 0
        // int fastReadResult = PIND & (1 << PD0); // Arduino Mega 2560 using pin 21 int 2

	//Call the appropriate method on the detector.
        //Beware: don't use the detectors methods directly as interrupt-handlers.
        //To make a long story short: You can't use class methods as interrupt-handlers 
        //(they loose there reference to their "this" object)
	if (fastReadResult != 0) {
		//state is up, must have been leading edge..
		detector.pickUpLeadingEdge();
	} else {
		//must have been trailing edge then...
		detector.pickUpTrailingEdge();
	}
}


//The setup function is called once at startup of the sketch
void setup() {
	Serial.begin(115200);

        // Pin 13 is used on all arduinos to give a little lightshow to the user
	pinMode(13, OUTPUT);

        // This is to make the interrupt-handler work on incoming signals!
	pinMode(2, INPUT);    // Arduino UNO using pin 2, int 0
        // pinMode(21, INPUT);// Arduino Mega 2560 using pin 21 int 2

        // This attaches the interrupthandler (and thereby the detector) to the interrupt
	attachInterrupt(0, &handler, CHANGE);   // Arduino UNO using pin 2, int 0
        //attachInterrupt(2, &handler, CHANGE); // Arduino Mega 2560 using pin 21 int 2
	//Do not use two interrupt handlers on the same pin!
	//(RISING, FALLING)... That didn't work for me.
	//Use CHANGE instead and do a fast read in the interrupt handler
	//to tell leading from trailing edges!

        // Now for the outputs...
        //For the UNO using pin 3-10 for output.
	for (int i = 3; i < 11; i++) {
        //On the Mega, best use pin 22-29 
        //for (int i = 22; i < 30; i++) {
		pinMode(i, OUTPUT);
	}
        // Tell the datahandler where to output to
        dataHandler.setFirstPin(3); // For UNO
        //dataHandler.setFirstPin(21); // For MEGA 2560
        
        // Finally attach the handler to the decoder.
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
