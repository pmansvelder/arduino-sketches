//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Arduino DCC Solenoid Switch Decoder.
// Author: Ruud Boer - January 2015
// This sketch turns an Arduino into a DCC decoder to drive max 8 dual coil solenoid switches.
// The DCC signal is optically separated and fed to pin 2 (=Interrupt 0). Schematics: www.mynabay.com
// Many thanks to www.mynabay.com for publishing their DCC monitor and -decoder code.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// IMPORTANT: GOTO lines 17 and 40 to configure some data!
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <DCC_Decoder.h>
#define kDCC_INTERRUPT 0

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FILL IN 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const byte maxaccessories=1; //The number of switches you want to control with this Arduino
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct
{
  int               address;         // Address to respond to
  byte              output;          // State of accessory: 1=on, 0=off (for internal use only)
  int               outputPin;       // Arduino output pin
  int               outputPin2;      // Arduino output pin2, used for solenoid junctions
  byte              highlow;         // State of outputpin: 1=HIGH, 0=LOW
  byte              highlow2;        // State of outputpin2: 1=HIGH, 0=LOW
  boolean           finished;        // Memory location that says the oneshot is finished
  boolean           finished2;       // Memory location that says the second oneshot (for solenoid) is finished
  int               durationMilli;   // ms flash time
  unsigned long     onMilli;         // for internal use
  unsigned long     offMilli;        // for internal use
} 
DCCAccessoryAddress;
DCCAccessoryAddress accessory[maxaccessories];

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization: COPY - PASTE the structure as many times as you have switches and fill in the values you want.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ConfigureDecoderFunctions() // The amount of accessories must be same as in line 26 above!
{
  accessory[0].address = 4;
  accessory[0].durationMilli = 250;
  accessory[0].outputPin = 12;
  accessory[0].outputPin2 = 13;
  accessory[0].highlow = 0; // Do not change this value
  accessory[0].highlow2 = 0; // Do not change this value
  accessory[0].finished = false; // Do not change this value
  accessory[0].finished2 = true; // Do not change this value
  accessory[0].output = 0; // Do not change this value

  // Setup output pins
  for(int i=0; i<maxaccessories; i++)
  {
    if( accessory[i].outputPin )
		{
      pinMode( accessory[i].outputPin, OUTPUT );
      digitalWrite( accessory[i].outputPin, LOW);
    }
    if( accessory[i].outputPin2 )
		{
      pinMode( accessory[i].outputPin2, OUTPUT );
      digitalWrite( accessory[i].outputPin2, LOW);
    }
  }
} // END ConfigureDecoderFunctions

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DCC packet handler 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BasicAccDecoderPacket_Handler(int address, boolean activate, byte data)
{
  // Convert NMRA packet address format to human address
  address -= 1;
  address *= 4;
  //address += 1;
  address += (data & 0x06) >> 1;

  boolean enable = (data & 0x01) ? 1 : 0;

  Serial.print(address);
  Serial.print(" - ");
  Serial.println(enable);

  for(int i=0; i<maxaccessories; i++)
	{
    if( address == accessory[i].address )
		{
      if( enable ) accessory[i].output = 1;
      else accessory[i].output = 0;
    }
  }
} // END BasicAccDecoderPacket_Handler

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup (run once)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() 
{ 
  Serial.begin(115200);
  DCC.SetBasicAccessoryDecoderPacketHandler(BasicAccDecoderPacket_Handler, true);
  ConfigureDecoderFunctions();
  DCC.SetupDecoder( 0x00, 0x00, kDCC_INTERRUPT );
  pinMode(2,INPUT_PULLUP); //Interrupt 0 with internal pull up resistor (can get rid of external 10k)
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW); //turn off Arduino led at startup

  for (int n=0; n<maxaccessories; n++) accessory[n].output = 0; //all servo's to min angle and functions to 0
} //END setup

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main loop (run continuous)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  static int addr = 0;

  DCC.loop(); // Loop DCC library

  if( ++addr >= maxaccessories ) addr = 0; // Bump to next address to test

  if (accessory[addr].output == 1)
	{
		if (!accessory[addr].highlow && !accessory[addr].finished)
		{
			accessory[addr].highlow = 1;
			accessory[addr].offMilli = millis() + accessory[addr].durationMilli;
		}
		if (accessory[addr].highlow && millis() > accessory[addr].offMilli)
		{
			accessory[addr].highlow = 0;
			accessory[addr].finished = true;
		}
		accessory[addr].finished2 = false;
  }

  else // output==0
  {
		accessory[addr].highlow=0;
		accessory[addr].finished = false;
		if (!accessory[addr].highlow2 && !accessory[addr].finished2)
		{
			accessory[addr].highlow2 = 1;
			accessory[addr].offMilli = millis() + accessory[addr].durationMilli;
		}
		if (accessory[addr].highlow2 && millis() > accessory[addr].offMilli)
		{
			accessory[addr].highlow2 = 0;
			accessory[addr].finished2 = true;
		}
  }

  if (accessory[addr].highlow) digitalWrite( accessory[addr].outputPin, HIGH);
  else digitalWrite( accessory[addr].outputPin, LOW);
  if (accessory[addr].highlow2) digitalWrite( accessory[addr].outputPin2, HIGH);
  else digitalWrite( accessory[addr].outputPin2, LOW);

} //END loop

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



