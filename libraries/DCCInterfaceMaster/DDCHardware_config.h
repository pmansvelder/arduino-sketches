/****************************************************************************
* Copyright (C) 2015 Philipp Gahtow
*
* DCC Waveform Timer Configuration
*
* DCC Master Interface can generate DCC Signal 
* either with Timer1 (16-bit) or with Timer2 (8-bit)
*
****************************************************************************/

//DCC output with true c command (10x faster!):
#define DCC_USE_TRUE_C

/******************************************/
//Set define for true c commands
#if defined(DCC_USE_TRUE_C)
//fix pin for DCC sginal out!
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) //Arduino MEGA (Pin 6)
#define DCC_PIN_TOGGLE() {PORTH ^= _BV(PH3);}		
#define DCC_PIN_LOW() {PORTH &= ~_BV(PH3);}

#elif defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644P__)  //Sanguino (Pin 12)
#define DCC_PIN_TOGGLE() {PORTD ^= _BV(PD4);}		
#define DCC_PIN_LOW() {PORTD &= ~_BV(PD4);}

#else //others Arduino like Atmega168, Atmega328 (Pin 6)
#define DCC_PIN_TOGGLE() {PORTD ^= _BV(PD6);}		
#define DCC_PIN_LOW() {PORTD &= ~_BV(PD6);}
#endif
#endif

//--------------------------------------------------------------------------------------
/*
NEM 670 – Ausgabe 2013:
Dauer des Teil-Einsbits: t = 58 µs & zulässige Toleranzen +/- 3 µs am Gleis
Dauer des Teil-Nullbits: t ≥ 100 µs
*/

//CONTROL What Timer we should use:
//#define DCC_USE_TIMER1		//USE 16-bit TIMER1
#undef DCC_USE_TIMER1		//USE 8-bit TIMER2

/******************************************/
//USE the Timer1 for the DCC Signal generation
#if defined(DCC_USE_TIMER1)
#undef DCC_USE_TIMER2

#define one_count 115 //58us = 115
#define zero_high_count 199 //100us = 199	!old: 116us = 228
#define zero_low_count 199  //100us

#define DCC_TMR_SIGNAL         TIMER1_COMPA_vect
#define DCC_INIT_COMPARATOR    TCCR1A
#define DCC_TMR_CONTROL_REG    TCCR1B
//turn on CTC mode in WGM12 and set CS11 for 8 prescaler in TCCR1B
#define DCC_TMR_CONTROL_SET()  {DCC_TMR_CONTROL_REG = 1 << WGM12 | 1 << CS11;}
#define DCC_TMR_COUNT_REG	   TCNT1
#define DCC_TMR_MATCH_INT()    {TIMSK1 |= (1 << OCIE1A);} //Compare Match Interrupt Enable
#define DCC_TMR_OUTP_CAPT_REG  OCR1A

#define DCC_TMR_OUTP_ONE_COUNT() {OCR1A = OCR1B = one_count;}
#define DCC_TMR_OUTP_ZERO_HIGH() {OCR1A = OCR1B = zero_high_count;}
#define DCC_TMR_OUTP_ZERO_LOW()  {OCR1A = OCR1B = zero_low_count;}

/******************************************/
//USE the Timer2 for the DCC Signal generation
#else
#define DCC_USE_TIMER2

#define one_count 0x8D  //58usec pulse length = 141
#define zero_high_count 56  //100us = 56		!old: 116us = 0x1B (27) pulse length
#define zero_low_count 56   //100us

#define DCC_TMR_SIGNAL         TIMER2_OVF_vect
#define DCC_INIT_COMPARATOR    TCCR2A
#define DCC_TMR_CONTROL_REG    TCCR2B
//Timer2 Settings: Timer Prescaler /8, mode 0
//Timmer clock = 16MHz/8 = 2MHz oder 0,5usec
#define DCC_TMR_CONTROL_SET()  {DCC_TMR_CONTROL_REG = 0 << CS22 | 1 << CS21 | 0 << CS20;}
#define DCC_TMR_COUNT_REG	   TCNT2
#define DCC_TMR_MATCH_INT()    {TIMSK2 = 1 << TOIE2;} //Overflow Interrupt Enable

//note that there is a latency so take the last time of Timer2 also:
#define DCC_TMR_OUTP_ONE_COUNT() {DCC_TMR_COUNT_REG = DCC_TMR_COUNT_REG + one_count; last_timer = one_count;}
#define DCC_TMR_OUTP_ZERO_HIGH() {DCC_TMR_COUNT_REG = DCC_TMR_COUNT_REG + zero_high_count; last_timer = zero_high_count;}
#define DCC_TMR_OUTP_ZERO_LOW() {DCC_TMR_COUNT_REG = DCC_TMR_COUNT_REG + zero_low_count; last_timer = zero_low_count;}

/******************************************/
#endif