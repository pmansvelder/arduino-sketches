/*
  ESP8266 Blink by Simon Peter
  Blink the blue LED on the ESP-01 module
  This example code is in the public domain

  The blue LED on the ESP-01 module is connected to GPIO1
  (which is also the TXD pin; so we cannot use Serial.print() at the same time)

  Note that this sketch uses LED_BUILTIN to find the pin with the internal LED
*/

void setup() {
  pinMode(15, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
}

// the loop function runs over and over again forever
void loop() {
  for (int i = 0; i < 1024; i++) {
    analogWrite(15, i);
    delay(20);
  }
  delay(2000);
  for (int i = 1023; i > 0 ; i--) {
    analogWrite(15, i);
    delay(20);
  } 
  delay(2000);
}
