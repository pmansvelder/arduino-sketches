// LDR library for line follower
//
// Up to 6 LDRs can be connected as follows
//
// GND --[ 10k ]-+--[ LDR ]--- 5V
//               |
//               Analog pin
//
// Set up LDR pins:

int LDRpin[6] = {A0, A1, A2, A3, A4, A5};

// set up initial values

int LDRValue[6] = { 0, 0, 0, 0, 0, 0 };

// calibrate these values as follows:
// - measure value in darkness (Vd)
// - measure value in full light (Vl)
// - threshold = Vd + (Vl-Vd) / 2

int LDRthreshold[6] = { 710, 710, 500, 500, 500, 500 };

// There are 2 functions:
// ShowLDRValue(LDR): LDR is the number of the LDR to measure (0…5), this shows the actual value, to calibrate the threshold
// LightOrDark(LDR) : LDR is the number of the LDR to measure, this returns true if light, false if dark

int ShowLDRValue(int LDR) {
  return analogRead(LDRpin[LDR]);
}

bool LightOrDark(int LDR) {
  return ( (analogRead(LDRpin[LDR])) > LDRthreshold[LDR] );
}
