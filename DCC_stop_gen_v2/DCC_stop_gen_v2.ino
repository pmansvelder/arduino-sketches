//
// Generating a DCC Signal using Timer2 and the Waveform Generator
//

#include <avr/io.h>
#include <avr/interrupt.h>


#define DRIVE_1() PORTB = B00010000
#define DRIVE_0() PORTB = B00001000

#define TRIG 2
#define TRIG_ON() PORTB |= _BV(TRIG)
#define TRIG_OFF() PORTB &= ~_BV(TRIG)
#define ENABLE_PIN 4

#define PATTERN_BYTES 18
#define COMMAND_BYTES 7


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

#define TIMER2_TARGET 114

#define SWITCH_PIN 7

byte cmd_state = WAIT_A_STR;

byte timer2_target = TIMER2_TARGET;
int timer2_delta = 0;

unsigned int mycount = 0;

byte dcc_bit_pattern[PATTERN_BYTES];
byte dcc_bit_pattern_buffered[PATTERN_BYTES];

byte c_bit;
byte dcc_bit_count_target;
byte dcc_bit_count_target_buffered;

byte c_buf;

char dcc_address = 11;
char dcc_speed = 6;
boolean dcc_forward = true;

boolean valid_frame = false;

char in;

boolean got_command = false;
boolean stop_command = true;

void setup() {

  // Setup Timer2


  configure_for_dcc_timing();

  pinMode(SWITCH_PIN, INPUT_PULLUP);

  pinMode(13, OUTPUT); // LED

  pinMode(12, OUTPUT); // +ve
  pinMode(11, OUTPUT); // -ve
  pinMode(10, OUTPUT); // trigger
  pinMode(ENABLE_PIN, OUTPUT);  // enable

  // Messing

  build_frame(3, true, 2);
  build_frame(5, false, 2);

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

void loop()
{
  if (digitalRead(SWITCH_PIN) == HIGH)
  {
    dcc_address = 11;
    dcc_forward = false;
    dcc_speed = 4;
    if (stop_command) { got_command = true; }
    stop_command = false;
  }
  else
  {
    dcc_address = 0;
    dcc_forward = true;
    dcc_speed = 0;
    if (not stop_command) { got_command = true; }
    stop_command = true;
  }

  if ( got_command ) {
    build_frame(dcc_address, dcc_forward, dcc_speed);
    got_command = false;
  }
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

void _write_test_frame( int testtype ) {
  char test_code;

  switch (testtype) {
    case 0: test_code = B00000000; break;
    case 1: test_code = B11111111; break;
  }

  for ( int i = 0; i < PATTERN_BYTES; i++ ) {
    dcc_bit_pattern[i] = test_code;
  }

  dcc_bit_count_target = 110;
  valid_frame = true;
}
