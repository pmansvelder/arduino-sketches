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


union {
  uint8_t BAR;
  struct {
    uint8_t  r1 : 2; // bit positions 0,1
    uint8_t  r2 : 2; // bit positions 2,3
    uint8_t  r3 : 2; // bit positions 4,5
    uint8_t  r4 : 2; // bit positions 6,7
    // total # of bits just needs to add up to the uint8_t size
  } bar;
} foo;

//Globally create a detector.
Detect145027 detector(Detect145027::LOCOMOTION_NOMINAL_PULSE_WIDTH_MICROS);
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

void PrintOnOff(boolean flag) {
                  if (flag)
                {
                  Serial.print("on");
                }
                else
                {
                  Serial.print("off");
                }
}

TurnOutDataHandler dataHandler;


unsigned long int lastTimeMillis = 0;

unsigned char led13=0;

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
        dataHandler.setMode(TurnOutDataHandler::MODETURNOUT);
	decoder.addHandler(&dataHandler);

}



// The loop function is called in an endless loop

void loop() {
        int Data = 0;
        int SpeedData = 0;
	if (detector.available()) {
		unsigned char* datagram = detector.getCurrentDatagram();

		Serial.print("Motorola Loc Data:");
		for (int i = 0; i < 18; i++) {
			Serial.print((int) datagram[i]);
                        if (i < 8)
                        {
                          Data = Data << 1;
                          Data += datagram[i];
                        }
                        if (i > 9)
                        {
                          SpeedData = SpeedData << 1;
                          SpeedData += datagram[i];
                        }
		}
                foo.BAR = Data;
                int Address = (foo.bar.r1 * 27) + (foo.bar.r2 * 9) + (foo.bar.r3 * 3) + (foo.bar.r4);
                int Light = datagram[9];
                Serial.print(" | Adress: ");
                Serial.print(Address,DEC);
                Serial.print(" | Light: ");
                PrintOnOff(Light);
                Serial.print(" | SpeedData: ");
                Serial.print(SpeedData,BIN);
                Serial.print(decoder.decodeDatagram(datagram),DEC);
                Serial.println();

		decoder.decodeDatagram(datagram);
	}

	if (Serial.available()) {
		Serial.read();
		Serial.println("I'm afraid I can't do that, Dave.");
	}

	dataHandler.handleTimeouts();

}
