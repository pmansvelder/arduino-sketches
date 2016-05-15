// sketch to detect a shortcircuit on the second rail 
// of a 3-rail H0 layout, then switch off the DCC signal
// to that rail.
// Rail layout (Marklin K-rail):
// ________ (first rail)
// . . . .  (middle contact)
// ________ (second rail)

int DCC_count = 0;  // counter for number of flanks
int max_flanks = 120;  // number of flanks to count before trying
int DCC_debounce = 8;  // number of tries to make sure we are on a positive flank
int DCC_pin = 7;  // sensor pin for digital signal (LOW = positive signal)
int rail_number = 1; // counter for which rail to test

// rail 1 connections
int sc_pin1 = 1;  // sensor pin for short circuit (HIGH = shortcircuit)
int railswitch1 = 13;  // output pin to switch off rail (HIGH = switch off)

// rail 2 connections
int sc_pin2 = 2;  // sensor pin for short circuit (HIGH = shortcircuit)
int railswitch2 = 12;  // output pin to switch off rail (HIGH = switch off)

// rail 3 connections
int sc_pin3 = 3;  // sensor pin for short circuit (HIGH = shortcircuit)
int railswitch3 = 11;  // output pin to switch off rail (HIGH = switch off)

// rail 4 connections
int sc_pin4 = 4;  // sensor pin for short circuit (HIGH = shortcircuit)
int railswitch4 = 10;  // output pin to switch off rail (HIGH = switch off)

// rail 5 connections
int sc_pin5 = 5;  // sensor pin for short circuit (HIGH = shortcircuit)
int railswitch5 = 9;  // output pin to switch off rail (HIGH = switch off)

// rail 6 connections
int sc_pin6 = 6;  // sensor pin for short circuit (HIGH = shortcircuit)
int railswitch6 = 8;  // output pin to switch off rail (HIGH = switch off)

void setup()
{
  // initialize serial communication at 115200 bits per second:
  pinMode(DCC_pin, INPUT);      // DCC signal, low = positive flank


  digitalWrite(DCC_pin, HIGH);  // Enable pullup resistor

  pinMode(sc_pin1, INPUT_PULLUP);       // Signal on 2nd rail, high = no signal  
  pinMode(railswitch1, OUTPUT);          // Output to opto to switch off signal to 2nd rail
  digitalWrite(railswitch1, LOW);       // Default is to switch off signal

  pinMode(sc_pin2, INPUT_PULLUP);       // Signal on 2nd rail, high = no signal  
  pinMode(railswitch2, OUTPUT);          // Output to opto to switch off signal to 2nd rail
  digitalWrite(railswitch2, LOW);       // Default is to switch off signal

  pinMode(sc_pin3, INPUT_PULLUP);       // Signal on 2nd rail, high = no signal  
  pinMode(railswitch3, OUTPUT);          // Output to opto to switch off signal to 2nd rail
  digitalWrite(railswitch3, LOW);       // Default is to switch off signal

  pinMode(sc_pin4, INPUT_PULLUP);       // Signal on 2nd rail, high = no signal  
  pinMode(railswitch4, OUTPUT);          // Output to opto to switch off signal to 2nd rail
  digitalWrite(railswitch4, LOW);       // Default is to switch off signal

  pinMode(sc_pin5, INPUT_PULLUP);       // Signal on 2nd rail, high = no signal  
  pinMode(railswitch5, OUTPUT);          // Output to opto to switch off signal to 2nd rail
  digitalWrite(railswitch5, LOW);       // Default is to switch off signal

  pinMode(sc_pin6, INPUT_PULLUP);       // Signal on 2nd rail, high = no signal  
  pinMode(railswitch6, OUTPUT);          // Output to opto to switch off signal to 2nd rail
  digitalWrite(railswitch6, LOW);       // Default is to switch off signal
}

void detectshort(int railpin, int switchpin) {
  if (digitalRead(railpin) == HIGH) {    // if no signal on rail
    if (digitalRead(DCC_pin) == LOW) {   // Are we still on the positive flank?
      if (digitalRead(railpin) == HIGH) {    // if no signal on rail
        digitalWrite(switchpin, HIGH);      // Then switch off signal to rail
      }
    }
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

  if (recursedetect(DCC_pin, DCC_debounce, LOW)) {
    DCC_count += 1;

    detectshort(sc_pin1, railswitch1);
    detectshort(sc_pin2, railswitch2);
    detectshort(sc_pin3, railswitch3);
    detectshort(sc_pin4, railswitch4);
    detectshort(sc_pin5, railswitch5);
    detectshort(sc_pin6, railswitch6);

    if (DCC_count >= max_flanks) {
      switch(rail_number) {
      case 1:
        digitalWrite(railswitch1, LOW);         // switch signal on to rail 1
        detectshort(sc_pin1, railswitch1);
      case 2:
        digitalWrite(railswitch2, LOW);         // switch signal on to rail 2
        detectshort(sc_pin2, railswitch2);
      case 3:
        digitalWrite(railswitch3, LOW);         // switch signal on to rail 3
        detectshort(sc_pin3, railswitch3);
      case 4:
        digitalWrite(railswitch4, LOW);         // switch signal on to rail 4
        detectshort(sc_pin4, railswitch4);
      case 5:
        digitalWrite(railswitch5, LOW);         // switch signal on to rail 5
        detectshort(sc_pin5, railswitch5);
      case 6:
        digitalWrite(railswitch6, LOW);         // switch signal on to rail 6
        detectshort(sc_pin6, railswitch6);
      default:
        rail_number = 0;
      }
      rail_number += 1;
      DCC_count = 0;
    }
  }

}

















































