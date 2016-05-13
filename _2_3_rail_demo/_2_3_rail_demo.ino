// sketch to detect a shortcircuit on the second rail 
// of a 3-rail H0 layout, then switch off the DCC signal
// to that rail.
// Rail layout (Marklin K-rail):
// ________ (first rail)
// . . . .  (middle contact)
// ________ (second rail)

long DCC_count = 0;
const int max_flanks = 10;
int DCC_pin = 2;
int sc_pin = 3;
int railswitch = 13;
int rail3 = 0;

void setup()
{
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  pinMode(DCC_pin, INPUT);      // DCC signal, low = positive flank
  pinMode(sc_pin, INPUT);       // Signal on 2nd rail, high = no signal
  pinMode(railswitch, OUTPUT);          // Output to opto to switch off signal to 2nd rail
  digitalWrite(DCC_pin, HIGH);  // Enable pullup resistor
  digitalWrite(sc_pin, HIGH);   // Enable pullup resistor
  digitalWrite(railswitch, LOW);       // Default is to switch off signal
}

// Basic loop should do the following:
// detect rising flank of DCC signal
// after a number of flanks, switch on second rail
// detect shortcut; if yes, turn off second rail

void loop(void)
{
  if (rail3 == 1) {
    if (digitalRead(DCC_pin) == LOW) {          // positive flank 
      digitalWrite(railswitch, LOW);         // switch signal on to rail
      if (digitalRead(sc_pin) == HIGH) {     // no signal on rail
        if (digitalRead(DCC_pin) == LOW) {   // Are we still on the positive flank?
          digitalWrite(railswitch, HIGH);      // Then switch off signal to rail 

        }  
        else {
          rail3 = 0;

        }
      }
    }
  }
  else {
    if (digitalRead(DCC_pin) == LOW) {          // positive flank 
      DCC_count += 1;
      if (DCC_count >= max_flanks) {
        digitalWrite(railswitch, LOW);         // switch signal on to rail
        if (digitalRead(sc_pin) == HIGH) {     // no signal on rail
          if (digitalRead(DCC_pin) == LOW) {   // Are we still on the positive flank?
            digitalWrite(railswitch, HIGH);      // Then switch off signal to rail
            rail3 = 1;
            delay(300);
          }
        }
        DCC_count = 0;
      }
    }
  }
}












