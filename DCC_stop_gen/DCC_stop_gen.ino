/*
  DCC test packet generator
  
  This program will repeatedly send a hard-coded DCC packet containing the following information:
  
  Locomotive address:    55
  Direction:             Forward
  Speed Step:            20
  
  It will pause between packets (during which time no other packets will be sent in this version.
  
  This version uses the highest level port control and timer functions and macros as a first test.
  
  Testing has shown that these are too slow and subject to jitter but it's still useful as a very
  high level reference for what we are trying to do.
  
 */

#define  DCC_PORT            8    // port number of arduino

#define  SWITCH_PORT         7    // port number to switch from test signal to stop signal

#define  INTER_PACK_DELAY    25    // milliseconds


// DCC manifest constants - all of these are hand tuned to hit the spec numbers based on using the
// digitalWrite() routines
//
// These routines also suffer from jitter that puts them outside of the DCC spec - direct port
// writes will probably be needed for more accurate control

#define  DCC_1_DELAY         53   // 58 microseconds for one phase of '1' bit - 6.5% long (was 54)
#define  DCC_0_DELAY         95    // 100 microseconds for one phase of '0' bit (was 96)

#define  PREAMBLE_BIT_COUNT  14    // number of 1's in packet preamble

#define  DCC_ADDR_SHORT      0
#define  DCC_ADDR_LONG       1

#define  DCC_DIR_FOR         1        
#define  DCC_DIR_REV         0

typedef struct {
  unsigned char  sAddr;
} DCC_SHORT_ADDR;

typedef struct {
  unsigned short lAddr;
} DCC_LONG_ADDR;

typedef union {
  unsigned int  val;
  
  struct {
    unsigned int  fixed:2;
    unsigned int  dir:1;
    unsigned int  compat:1;
    unsigned int  speed:4;
  } bits;
} DCC_BASIC_PACKET;

DCC_SHORT_ADDR    testAddr = { 11 }; // Address 11
DCC_SHORT_ADDR    stopAddr = { 0 }; // Address 7
DCC_BASIC_PACKET  stopInst;
DCC_BASIC_PACKET  testInst;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin as an output.
  pinMode(DCC_PORT, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  pinMode(SWITCH_PORT, INPUT_PULLUP);
  digitalWrite(DCC_PORT, LOW);  // steady state condition
  
  // setup test instruction packet
  stopInst.bits.fixed  = 0x01;
  stopInst.bits.dir    = DCC_DIR_FOR; // Forward Direction
  stopInst.bits.compat = 1;
  stopInst.bits.speed  = 0;  // stop loco

  // setup test instruction packet
  testInst.bits.fixed  = 0x01;
  testInst.bits.dir    = DCC_DIR_FOR;
  testInst.bits.compat = 1;
  testInst.bits.speed  = 6;
}

//  clock out the appropriate DCC bit by toggling the DCC port high
void dccBit(int bit)

{
  switch (bit) {
    
    case 1:
      digitalWrite(DCC_PORT, HIGH);
      delayMicroseconds(DCC_1_DELAY);
      digitalWrite(DCC_PORT, LOW);
      delayMicroseconds(DCC_1_DELAY);
      break;

    case 0:
      digitalWrite(DCC_PORT, HIGH);
      delayMicroseconds(DCC_0_DELAY);
      digitalWrite(DCC_PORT, LOW);
      delayMicroseconds(DCC_0_DELAY);
      break;
  }
}



//  dccByte
//
//  clock out a DCC byte

void dccByte(unsigned char byte) {

int  i;

    for (i = 0 ; i < 8 ; i++) {
      if (byte & 0x80) {          // check the high bit
        dccBit(1);
      } else {
        dccBit(0);
      }
      byte <<= 1;                 // shift for the ext bit
    }  
}


//  dccPacket
//
//  send the DCC packet
//
//  supports both long and short addresses

void dccStopPacket(int addrType, unsigned char *addr, int dataCount, unsigned char *data) {

int            i;

  for (i = 0 ; i < PREAMBLE_BIT_COUNT ; i++) {
    dccBit(1);      // send the preamble bits
  }
  
  dccBit(0);        // end of preamble
  
  dccByte(*addr);
  
  dccBit(0);        // end of address
  
  dccByte(*data);
    
  dccBit(0);        // end of instructions - start of error detection value
  
  dccByte(*data);   // for broadcast stop, error detection equals data byte

/*
111111111111 0 00000000 0 01DC000S 0 EEEEEEEE 1

Preamble Byte One Byte Two Byte Three (Error Detection Data Byte)

A three byte packet, whose first byte contains eight "0"s, whose second byte contains a specific stop command
and whose third and final byte contains an error byte that is identical to the second byte of the packet,
is defined as a Digital Decoder Broadcast Stop Packet. Upon receiving this packet where bit zero of byte two (S)
contains a value of "0", digital decoders intended to control a locomotive's motor shall bring the locomotive to a stop.
 */
  
  
  dccBit(1);        // end of error detection
}

//  dccPacket
//
//  send the DCC packet
//
//  supports both long and short addresses

void dccPacket(int addrType, unsigned char *addr, int dataCount, unsigned char *data) {

int            i;
unsigned char  local;

  for (i = 0 ; i < PREAMBLE_BIT_COUNT ; i++) {
    dccBit(1);      // send the preamble bits
  }
  
  dccBit(0);        // end of preamble
  
  if (addrType == DCC_ADDR_SHORT) {    // process it as a short address (implies number of address bytes)
    dccByte(*addr);
  } else {                             // it's a long address - 2 bytes to clock out
  }
  
  dccBit(0);        // end of address

  dccByte(*data);

  dccBit(0);        // end of instructions - start of error detection value
  
  dccByte((unsigned char) 0x7F);    // hard coded for now
  
  dccBit(1);        // end of error detection
}


void loop() {

  if (digitalRead(SWITCH_PORT) == LOW) {
    dccStopPacket(DCC_ADDR_SHORT, (unsigned char *)&stopAddr, 1, (unsigned char *)&stopInst);
    digitalWrite(13, LOW);
  }
  else {
    dccPacket(DCC_ADDR_SHORT, (unsigned char *)&testAddr, 1, (unsigned char *)&testInst);
    digitalWrite(13, HIGH);    
  }
  dccByte(0);
  
  delay(10);  //delay 100 microseconds (was 1000)
}


