//
// Generating a DCC Signal using Timer2 and the Waveform Generator
//

#include <avr/io.h>
#include <avr/interrupt.h>

// PORTB maps to Arduino digital pins 8 to 13 The two high bits (6 & 7) map to the crystal pins and are not usable

#define DRIVE_1() PORTB = B00010000 // so this sets port 12 to 1, other ports (8,9,10,11,13) to 0
#define DRIVE_0() PORTB = B00001000 // so this sets port 11 to 1, other ports (8,9,10,12,13) to 0

#define SWITCH_PIN 7
#define LED_PIN 13
#define VE_PLUS_PIN 12
#define VE_MIN_PIN 11
#define TRIGGER_PIN 10

#define TRIG 2
#define TRIG_ON() PORTB |= _BV(TRIG)
#define TRIG_OFF() PORTB &= ~_BV(TRIG)
#define ENABLE_PIN 4

//#define PATTERN_BYTES 22 // for DCC
#define PATTERN_BYTES 36 // for motorola

//#define TIMER2_TARGET 114 // for DCC
#define TIMER2_TARGET 56 // for Motorola

#define WAIT_A_STR  0 /* or testmode, or timer2 delta*/
#define WAIT_A_INT  1
#define WAIT_A_INT_0  100
#define WAIT_A_INT_1  101
#define WAIT_A_INT_2  102
#define WAIT_SEP_1  2
#define WAIT_DIR    3
#define WAIT_SEP_2  4
#define WAIT_S_STR  5
#define WAIT_S_INT  6
#define WAIT_TERM   7
#define WAIT_T_INT  8  /* testmode */
#define WAIT_D_INT_0 9
#define WAIT_D_INT_1 10

const byte stop_pattern[] = { B10101010, B10101010, B10101010, B11001010, B11001100, // 1111 1111 1111 11 0  <00
                              B11001100, B11001100, B11001100, B11001100, B11001010, // 00 00 00> 0 <0 11 0
                              B11001100, B11001100, B11001100, B11001010, B11001100, // 00 00> 0 <0 11 0 00
                              B11001100, B00000010, B00000000 // 00> 1
                            };
const byte stop_pattern_double[] = { B11001100, B11001100, B11001100, B11001100, B11001100, B11001100, B11001100, // preamble (14 x 1)
                                     B11110000, // start bit (0)
                                     B11110000, B11110000, B11110000, B11110000, B11110000, B11110000, B11110000, B11110000, // 8 zero bits
                                     B11110000, // start bit
                                     B11110000, B11001100, B11110000, B11110000, B11110000, B11110000, B11110000, // 0 11 0 0000
                                     B11110000, // start bit
                                     B11110000, B11001100, B11110000, B11110000, B11110000, B11110000, B11110000, // 0 11 0 0000
                                     B00001100, B00000000, B00000000, B00000000, B00000000 // 1 0
                                   };

const byte test_pattern_old[] = { B10101010, B10101010, B10101010, B11001010, B11001100, // 1111 1111 1111 11 0 <00
                                  B11001100, B10110010, B00110010, B11001011, B11001100, // 00 101 100 1000
                                  B00110010, B00110011, B11001011, B10101100, B11101010, // 100 00 100 11111 0
                                  B11001100, B00000010, B00000000 // 01
                                };

const byte test_pattern[] = { B10101010, B10101010, B10101010, B11001010,
                              B11001100, B11001100, // 1111 1111 1111 11 0 <00 00
                              B10110010, B00110010,
                              B00101011, B11001011, B00110010, B10110011, // 101 1> 0 0 110 10 10 0 <01
                              B00110010, B00110011, B00101011, B00000000, B00000000, B00000000 // 10 0 00 1> 1
                            };

const byte test_pattern_double[] = { B11001100, B11001100, B11001100, B11001100,
                                     B11001100, B11001100, B11001100, // preamble (14 x 1)
                                     B11110000, // start bit (0)
                                     B11110000, B11110000, B11110000, B11110000,
                                     B00001100, B11001111, B00001100, B00001111, // adress: 00 00 1011 0
                                     B11001111, B00001100, B11001111, B11110000,
                                     B00001100, B00001111, B00001111, // data: 01101010 0
                                     B11001111, B00001100, B00001111, B00001111, B00001111, B11001111, B00001100,
                                     B00000000, B00000000, B00000000, B00000000// checksum: 01101101 1
                                   };

byte cmd_state = WAIT_A_STR;

byte timer2_target = TIMER2_TARGET;
int timer2_delta = 0;

unsigned int mycount = 0;

byte dcc_bit_pattern[PATTERN_BYTES];
byte dcc_bit_pattern_buffered[PATTERN_BYTES];

int c_bit;
int dcc_bit_count_target;
int dcc_bit_count_target_buffered;

int c_buf;

byte dcc_address = 11;
byte dcc_speed = 2;
boolean dcc_forward = true;

boolean valid_frame = false;

char in;

boolean got_command = false;
boolean stop_command = true;

void printBits(byte myByte) {
  for (byte mask = 0x80; mask; mask >>= 1) {
    if (mask & myByte)
      Serial.print('1');
    else
      Serial.print('0');
  }
  Serial.println();
}

void setup() {

  // Messing
  Serial.begin(9600);
  show_bit_pattern();

  // Setup Timer2

  configure_for_dcc_timing();

  pinMode(SWITCH_PIN, INPUT_PULLUP);

  pinMode(LED_PIN, OUTPUT); // LED

  pinMode(VE_PLUS_PIN, OUTPUT); // +ve
  pinMode(VE_MIN_PIN, OUTPUT); // -ve
  pinMode(TRIGGER_PIN, OUTPUT); // trigger
  pinMode(ENABLE_PIN, OUTPUT);  // enable

  for (int i; i < PATTERN_BYTES; i++) {
    dcc_bit_pattern[i] = 0;
  }

  load_new_frame();
  c_buf = 0;
  digitalWrite(ENABLE_PIN, HIGH);
  sei();  // Enable interrupts
  got_command = true;
}

ISR( TIMER2_COMPA_vect ) {
  TCNT2 = 0; // Reset Timer2 counter to divide...

  boolean bit_ = bitRead(dcc_bit_pattern_buffered[c_buf >> 3], c_buf & 7 );

  if ( bit_ ) {
    DRIVE_1();
  } else {
    DRIVE_0();
  }

  /* Trigger for start of packet */
  /*  if(c_buf == (dcc_bit_count_target_buffered >> 1) ){
      TRIG_OFF();
    } else if( c_buf == dcc_bit_count_target_buffered ) {
      TRIG_ON();
    }
  */

  if (c_buf == 0 ) {
    TRIG_ON();
  } else {
    TRIG_OFF();
  }

  /* Now update our position */
  if (c_buf == dcc_bit_count_target_buffered) {
    c_buf = 0;
    load_new_frame();
  } else {
    c_buf++;
  }
};

/**
   This function is called from within the interrupt handler
   so it cannot use Serial.print functions without failing.
*/
void load_new_frame() {
  if ( valid_frame ) {
    for (int i = 0; i < PATTERN_BYTES; i++) {
      dcc_bit_pattern_buffered[i] = dcc_bit_pattern[i];
    }
    dcc_bit_count_target_buffered = dcc_bit_count_target - 1;
    valid_frame = false;
  }
};

void show_bit_pattern() {
  for ( int i = 0; i < PATTERN_BYTES; i++) {
    printBits(dcc_bit_pattern[i]);
  }
  Serial.print(dcc_bit_count_target);
  Serial.println(" bits.");
}

void configure_for_dcc_timing() {

  /* DCC timing requires that the data toggles every 58us
    for a '1'. So, we set up timer2 to fire an interrupt every
    58us, and we'll change the output in the interrupt service
    routine.

    Prescaler: set to divide-by-8 (B'010)
    Compare target: 58us / ( 1 / ( 16MHz/8) ) = 116
  */

  // Set prescaler to div-by-8
  bitClear(TCCR2B, CS22);
  bitSet(TCCR2B, CS21);
  bitClear(TCCR2B, CS20);

  // Set counter target
  OCR2A = timer2_target;

  // Enable Timer2 interrupt
  bitSet(TIMSK2, OCIE2A);
}

/* --------------------------------------------------------------
    DCC Packet setup
   --------------------------------------------------------------
*/

void bit_pattern(byte mybit) {

  bitClear(dcc_bit_pattern[c_bit >> 3], c_bit & 7 );
  c_bit++;

  if ( mybit == 0 ) {
    bitClear(dcc_bit_pattern[c_bit >> 3], c_bit & 7 );
    c_bit++;
  }

  bitSet(dcc_bit_pattern[c_bit >> 3], c_bit & 7 );
  c_bit++;

  if ( mybit == 0 ) {
    bitSet(dcc_bit_pattern[c_bit >> 3], c_bit & 7 );
    c_bit++;
  }

}

/* DCC pattern for this byte: MSB first */
void byte_pattern( byte mybyte ) {
  for ( int i = 7; i >= 0; i-- ) {
    bit_pattern( byte( bitRead( mybyte, i ) ) );
  }
};

/* Preamble pattern, 14 '1's */
void preamble_pattern() {
  for ( byte i = 0; i < 14; i++ ) {
    bit_pattern(1);
  }
}

/* Command Pattern */
byte calc_command_pattern( boolean dcc_forward, byte dcc_speed ) {
  byte command_byte;
  command_byte = B01000000 | ( dcc_forward << 5 ) | ( dcc_speed & B00011111 );
  return command_byte;
};

/* Build the DCC frame */
void build_frame(byte dcc_address, boolean dcc_forward, byte dcc_speed) {

  valid_frame = false;
  byte dcc_command = calc_command_pattern( dcc_forward, dcc_speed );
  byte dcc_checksum = dcc_command ^ dcc_address;

  // Build up the bit pattern for the DCC frame
  _build_frame(dcc_address, dcc_command, dcc_checksum);

  valid_frame = true;
};

void build_idle_frame() {
  valid_frame = false;
  _build_frame(B11111111, B00000000, B11111111);
  valid_frame = true;
};

void build_reset_frame() {
  valid_frame = false;
  _build_frame(B00000000, B00000000, B00000000);
  valid_frame = true;
};

void _build_frame( byte byte1, byte byte2, byte byte3) {

  // Build up the bit pattern for the DCC frame
  c_bit = 0;
  preamble_pattern();

  bit_pattern(0);
  byte_pattern(byte1); /* Address */

  bit_pattern(0);
  byte_pattern(byte2); /* Speed and direction */

  bit_pattern(0);
  byte_pattern(byte3); /* Checksum */

  bit_pattern(1);

  dcc_bit_count_target = c_bit;

};


void loop2()
{
  if (digitalRead(SWITCH_PIN) == HIGH)
  {
    dcc_address = 11;
    dcc_forward = true;
    dcc_speed = 6;
    if (stop_command) {
      Serial.println("Drive command");
      got_command = true;
    }
    stop_command = false;
  }
  else
  {
    dcc_address = 0;
    dcc_forward = true;
    dcc_speed = 0;
    if (not stop_command) {
      Serial.println("Stop command");
      got_command = true;
    }
    stop_command = true;
  }

  if ( got_command ) {
    build_frame(dcc_address, dcc_forward, dcc_speed);
    show_bit_pattern();
    Serial.println(dcc_bit_count_target);
    got_command = false;
  }
}

void loop_singe() // single bit, DCC spacing (116 us)
{
  if (digitalRead(SWITCH_PIN) == HIGH)
  {
    if (stop_command) {
      valid_frame = false;
      memcpy( dcc_bit_pattern, test_pattern, PATTERN_BYTES * sizeof(byte) );
      dcc_bit_count_target = 118; // was 118
      valid_frame = true;
      got_command = true;
    }
    stop_command = false;
  }
  else
  {
    if (not stop_command) {
      valid_frame = false;
      memcpy( dcc_bit_pattern, stop_pattern, PATTERN_BYTES * sizeof(byte) );
      dcc_bit_count_target = 130; // was 130
      valid_frame = true;
      got_command = true;
    }
    stop_command = true;
  }

  if ( got_command ) {
    show_bit_pattern();
    got_command = false;
  }
}

void loop() // double bit, MM spacing (58us)
{
  if (digitalRead(SWITCH_PIN) == HIGH)
  {
    if (stop_command) {
      valid_frame = false;
      Serial.println("Drive command");
      memcpy( dcc_bit_pattern, test_pattern_double, PATTERN_BYTES * sizeof(byte) );
      dcc_bit_count_target = 236;
      valid_frame = true;
      got_command = true;
    }
    stop_command = false;
  }
  else
  {
    if (not stop_command) {
      valid_frame = false;
      Serial.println("Stop command");
      memcpy( dcc_bit_pattern, stop_pattern_double, PATTERN_BYTES * sizeof(byte) );
      dcc_bit_count_target = 264;
      valid_frame = true;
      got_command = true;
    }
    stop_command = true;
  }

  if ( got_command ) {
    show_bit_pattern();
    got_command = false;
  }
}
