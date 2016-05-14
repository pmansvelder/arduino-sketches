// sketch to detect a shortcircuit on the second rail 
// of a 3-rail H0 layout, then switch off the DCC signal
// to that rail.
// Rail layout (Marklin K-rail):
// ________ (first rail)
// . . . .  (middle contact)
// ________ (second rail)

long DCC_count = 0;
const int max_flanks = 20;
int DCC_pin = 2;  // sensor pin for digital signal (LOW = positive signal)
int sc_pin1 = 3;  // sensor pin for short circuit (HIGH = shortcircuit)
int railswitch1 = 13;  // output pin to switch off rail (HIGH = switch off)
int delaytime = 1; //delaytime in microseconds to stabilize flank
int lastDCCstate = HIGH; // default state for DCC is no signal
int DCCstate = HIGH;
unsigned long lastDebounceTime = 0;
int debounce;

void setup()
{
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  pinMode(DCC_pin, INPUT);      // DCC signal, low = positive flank
  pinMode(sc_pin1, INPUT);       // Signal on 2nd rail, high = no signal
  pinMode(railswitch1, OUTPUT);          // Output to opto to switch off signal to 2nd rail
  digitalWrite(DCC_pin, HIGH);  // Enable pullup resistor
  digitalWrite(sc_pin1, HIGH);   // Enable pullup resistor////
  digitalWrite(railswitch1, LOW);       // Default is to switch off signal
}

void detectshort(int railpin, int switchpin) {
  if (digitalRead(railpin) == HIGH) {    // if no signal on rail
    digitalWrite(switchpin, HIGH);      // Then switch off signal to rail
  }
}

boolean recursedetect(int pin, int level, int expect) {
  while (level >= 0) {
    if (digitalRead(pin) == expect) {
      level -= 1;
    }
    else { 
      return false;
    }
  }
  return true;
}

// Basic loop should do the following:
// detect rising flank of DCC signal: serverl polls, ca. 3 x 10ms
// detect short: if yes, turn off rail
// randomize polling in case of multiple railds
// switch on second rail
// detect shortcut; if yes, turn off second rail

void loop()
{ 
  // flank detection  

  if (recursedetect(DCC_pin, 12, LOW)) {
    DCC_count += 1;
    if (digitalRead(sc_pin1) == HIGH) {
      if (digitalRead(DCC_pin) == LOW) {   // Are we still on the positive flank?
        digitalWrite(railswitch1, HIGH);
      }
    }

    if (DCC_count >= max_flanks) {
      digitalWrite(railswitch1, LOW);         // switch signal on to rail
      if (recursedetect(sc_pin1,1, HIGH)) {
        if (digitalRead(DCC_pin) == LOW) {   // Are we still on the positive flank?
          digitalWrite(railswitch1, HIGH);
        }
      }
      DCC_count = 0;
    }
  }

}








































