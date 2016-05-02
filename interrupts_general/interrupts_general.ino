#include <avr/interrupt.h>
                              //
                              
volatile long TimerCount = 0;
volatile int val,status;
volatile int RetryDelay;

void setup()
{
    // initialize serial communication at 115200 bits per second:
    Serial.begin(115200);
    pinMode(2, INPUT);
    pinMode(13, OUTPUT);
    pinMode(3, INPUT);
    digitalWrite(3, HIGH);    // Enable pullup resistor
    digitalWrite(2, HIGH);    // Enable pullup resistor
//    attachInterrupt(0, pin2ISR, RISING);
    digitalWrite(13, HIGH);
}
                              //
void loop(void)
{
//    Serial.print("Aantal interrupts: ");
//    Serial.println(TimerCount);
//    val = digitalRead(3);
//    Serial.println(digitalRead(3));
    digitalWrite(13, LOW);    // Zet spanning op rail
    if (digitalRead(2) == LOW) {  // positief signaal 
      if (digitalRead(3) == HIGH) { // geen signaal op rail
      digitalWrite(13, HIGH);    // Haal spanning van rail
      Serial.println("Kortsluiting gedetecteerd");
      RetryDelay = 3000;
      }
      else {
        RetryDelay = 0;
      }
    }
    TimerCount = 0;
    delay(RetryDelay);
    
}
                              //
// Interrupt Service Routine attached to INT0 vector
void pin2ISR()
{
    val = digitalRead(3);
//   digitalWrite(13, !digitalRead(13));    // Toggle LED on pin 13
    TimerCount += 1;
}
