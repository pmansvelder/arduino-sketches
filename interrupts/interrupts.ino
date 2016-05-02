#include <avr/interrupt.h>
                              //
                              
volatile long TimerCount = 0;

void setup(void)
{
    // initialize serial communication at 115200 bits per second:
    Serial.begin(115200);
    pinMode(2, INPUT);
    pinMode(13, OUTPUT);
    digitalWrite(2, HIGH);    // Enable pullup resistor
    sei();                    // Enable global interrupts
    EIMSK |= (1 << INT0);     // Enable external interrupt INT0
    EICRA |= (1 << ISC01);    // Trigger INT0 on falling edge
    digitalWrite(13, HIGH);
}
                              //
void loop(void)
{
    Serial.print("Aantal interrupts: ");
    Serial.println(TimerCount);
    TimerCount = 0;
    delay(500);
    
}
                              //
// Interrupt Service Routine attached to INT0 vector
ISR(INT0_vect)
{
    digitalWrite(13, !digitalRead(13));    // Toggle LED on pin 13
    TimerCount += 1;
}
