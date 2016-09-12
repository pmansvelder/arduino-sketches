//--------------------------------------------------------------
/*

  DCC Decoder Interface
  
*/

#if defined(DECODER)

#define maxSaved 8        //numbers of Data that is saved to compare a new packet

  //Eingänge:
#define decDCCPin 2      //nur auf Pin 2 geht die attachInterrupt Funktion!

//Variablen für DCC Erkennung
unsigned char clockSelectBits;    //for Timer4
#define RESOLUTION 65536          //Timer4 is 16 bit

byte countone = 0;    //Zähle die high so das man eine Präambel erkennen kann (min. 10 high)
boolean getdata = false;     //Es werden DCC Daten gelesen und im Array data abgespeichert
boolean dataReady = false;   //Es wurden DCC Daten vollständig eingelesen
byte decDCCData[5];          //eingelesene Byte-Werte nach einer Präambel
byte datalength = 0;  //Position in data wo gerade der nächste Wert hinkommt
byte countbit = 0;    //Zähle Bits, nach 8 Bit wurde ein Byte gelesen und setzte dann die Variable auf 0
byte dxor = 0;        //xor Prüfsumme
unsigned int dccAdr = 0;          //Adresse

#define DCC_SHORT_ADDRESS           0x00
#define DCC_LONG_ADDRESS            0x01
#define DCC_ACC_ADDRESS             0x02

byte SavedXOR[maxSaved];        //The XOR of the last circles
uint16_t SavedADR[maxSaved];    //The Adress and kind of data send in the last circles
byte SavedCount = 0;

//****************************************************************  
void Timer4_initialize(long microseconds) {
  TCCR4A = 0;                 // clear control register A 
  TCCR4B = _BV(WGM43);        // set mode 8: phase and frequency correct pwm, stop the timer
  
  long cycles = (F_CPU / 2000000) * microseconds;                                // the counter runs backwards after TOP, interrupt is at BOTTOM so divide microseconds by 2
  if(cycles < RESOLUTION)              clockSelectBits = _BV(CS40);              // no prescale, full xtal
  else if((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS41);              // prescale by /8
  else if((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS41) | _BV(CS40);  // prescale by /64
  else if((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS42);              // prescale by /256
  else if((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS42) | _BV(CS40);  // prescale by /1024
  else        cycles = RESOLUTION - 1, clockSelectBits = _BV(CS42) | _BV(CS40);  // request was out of bounds, set as maximum
  ICR4 = cycles;                                                     // ICR1 is TOP in p & f correct pwm mode
  TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));  // clears all clock selects bits 

  //attachInterrupt:
  TIMSK4 = _BV(TOIE4);                                     // sets the timer overflow interrupt enable bit
  sei();                                                   // ensures that interrupts are globally enabled
}

//****************************************************************  
//Interrupt funktion auf dccPin
void DCCDecoder_data() {
  TCCR4B |= clockSelectBits;  //Timer starten
}
 
//****************************************************************   
//Timer4 nach Intterrupt auf DCC-Pin
ISR(TIMER4_OVF_vect)          // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
  //Timer anhalten:
  TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));     // clears all clock selects bits 
  int State = digitalRead(decDCCPin);
  if (getdata == true) {
    countbit += 1;    //Abzählen der Bytes, also jeweils 8 Bit
  }
  if (State == LOW) {      //1-Bit gelesen
    countone += 1;            //Zählen der 1-Bit für Präambel erforderlich
    if (getdata == true && countbit <= 8) {    //eingelesenen Bitwert im Array Speichern
      bitWrite(decDCCData[datalength], 8-countbit, 1);    //Speichert das ein 1-Bit gelesen wurde
    }
    if (countbit > 8) {        //End-Bit gelesen.
      countbit = 0; 
      countone = 0;            //lösche Anzahl gelesener 1-Bit
      getdata = false;        //Stop des Einlesen der Daten
      //XOR Prüfen:
        //Prüfen von XOR und letztes Byte
      if (decDCCData[datalength] != dxor)
        return;    //verwerfen!
        
      while (datalength < 4) {  //Löschen des leeren Bereichs am Ende
        datalength++;
        decDCCData[datalength] = 0;
      }
      dataReady = true;  //Fertig, Daten Auswerten!
    }
  }  //Ende 1-Bit
  else {                  //0-Bit gelesen
    if (getdata == true && countbit <= 8) {    //eingelesenen Bitwert im Array Speichern
      bitWrite(decDCCData[datalength], 8-countbit, 0);    //Speichert das ein 0-Bit gelesen wurde
    }
    if (countone > 10) {   //Präambel erkannt ab hier Daten lesen. (Es wurden mind. 10 HIGH erkannt)
      getdata = true;     //Einlesen der Bitwerte in Array aktivieren.
      datalength = 0;    //Position im Array an der die Bitwerte gespeichert werden.
      countbit = 0;       //beginne Bits zu zählen. Auswertung von 1 Byte Blöcken ermöglichen nach 8 Bit.
      dxor = 0;          //XOR zurücksetzten
    }
    if (countbit > 8) {    //Null-Bit gelesen. (Immer nach 1 Byte)
      countbit = 0;
      dxor = dxor ^ decDCCData[datalength];  //XOR bestimmen!
      datalength += 1;  //Bitweise im Array weitergehen.
    }
    countone = 0;    //Es wurde ein 0-Bit gelesen, lösche Anzahl gelesener 1-Bit
  }  //Ende 0-Bit
}

//****************************************************************  
//Auswerten der eingelesenen Bitwerte.
void DCCDecoder_anaylse() {
  /*
    Beschreibung der Einträge im data Array:
      decDCCData[0]   : 1. Byte nach der Präambel (Adresse)
      Bit 8     : Null-Bit
      decDCCData[1]   : 2. Byte (long Adresse oder Daten)
      Bit 17    : Null-Bit
      decDCCData[2]   : 3. Byte (bei datalength == 26 => CRC)
      Bit 26    : Null-Bit (bei datalength == 26 => End-Bit)
      decDCCData[3]   : 4. Byte (wenn vorhanden, long Adr. oder long Data)
      Bit 35    : End-Bit (wenn vorhanden, long Adr. oder long Data)
      decDCCData[4]   : 5. Byte (wenn vorhanden, long Adr. und long Data)
      Bit 44    : End-Bit (wenn vorhanden, long Adr. und long Data)
  */
    //1.1 Bestimmen kurzen Adresse (bis 127)
  if (bitRead(decDCCData[0],7) == 0) 
    dccAdr = decDCCData[0];

  //1.2 Bestimmen der langen Adresse mit 14 Bits also 16384
  byte verschub = 0;
  if (decDCCData[0] >> 6 == B11) {
    dccAdr = word(decDCCData[0] & B00111111, decDCCData[1]);
    verschub = 1; //langes Datenpacket
    if (dccAdr >= 12000) {  //idle
      return;
    }
  }

  //1.3 Check if already worked on this packet:
  uint16_t thisADR = (dccAdr | 0xC000) & ((decDCCData[1+verschub] << 8)  | 0x3FFF) ;
  for (byte i = 0; i < maxSaved; i++) {
    if (SavedXOR[i] == dxor && SavedADR[i] == thisADR) 
      return;  //nothing to do here - same packet already send! 
    if (SavedADR[i] == thisADR) {
      SavedCount = i;  //Data change - overwrite this!
      break;  
    }
  }
  SavedXOR[SavedCount] = dxor;  //SAVE
  SavedADR[SavedCount] = thisADR;
  SavedCount++;
  if (SavedCount == maxSaved) {
    SavedCount = 0;
  }
  
  //1.4 Bestimmen der Weichenadresse
  byte dccAPort = 0;
  if ((decDCCData[0] >> 6 == B10) && (decDCCData[1] >> 7 == 1)) {  
    dccAdr = decDCCData[0] & B00111111;  //1. Adressteil
    dccAdr = (((decDCCData[1] & B01110000) << 2) | B00111111) & dccAdr; //2. Adressteil
    dccAPort = (decDCCData[1] >> 1) & B11;  //Port bestimmen, bis zu 4 möglich
//    dccType = DCC_ACC_ADDRESS;
/*    Serial.print("AA:");
    Serial.print(dccAdr);
    Serial.print(" P:");
    Serial.print(dccAPort);
    if (decDCCData[1] & B1 == 1)
      Serial.print(" activ");
*/      
    dcc.setBasicAccessoryPos((dccAdr*4)+dccAPort,decDCCData[1] & B1);
    return;
  }
  else { //dccType != DCC_ACC_ADDRESS
      if (((decDCCData[1+verschub] >> 5) & B111) == B100) {  //Funktionen F0 bis F4
        dcc.setFunctions0to4(dccAdr, decDCCData[1+verschub] & 0x1F);
        return;
      }
      if (((decDCCData[1+verschub] >> 4) & B1111) == B1011) {  //Funktionen F5 bis F8
        dcc.setFunctions5to8(dccAdr, decDCCData[1+verschub] & 0x0F);
        return;
      }
      if (((decDCCData[1+verschub] >> 4) & B1111) == B1010) {  //Funktionen F9 bis F12
        dcc.setFunctions9to12(dccAdr, decDCCData[1+verschub] & 0x0F);
        return;
      }
      if (decDCCData[1+verschub] >> 6 == B01) {  //Fahrgeschwindigkeit/Fahrtrichtung 14 oder 28 Stufen
        uint8_t Vspeed = (decDCCData[1+verschub] & B1111) << 1; 
        bitWrite(Vspeed,0,bitRead(decDCCData[1+verschub],4));  //additional speed bit
        if (Vspeed > 3) {
          Vspeed = Vspeed - 3;    //4-31 to 1-28
          Vspeed = map(Vspeed, 1, 28, 1, 126);    //0-126
        }
        if (bitRead(decDCCData[1+verschub],5) == 0) 
          Vspeed = Vspeed + 128;
        dcc.setSpeed128(dccAdr, Vspeed);          
        return;
      }
      if (decDCCData[1+verschub] == B00111111) {  //126 Fahrstufen Auswerten
        uint8_t Vspeed = decDCCData[2+verschub];
        if (Vspeed > 1 && Vspeed < 128)
          Vspeed--;
        if (Vspeed > 129)
          Vspeed--;  
        dcc.setSpeed128(dccAdr, Vspeed);  
        return;
      }
/*      if (decDCCData[1+verschub] == B11011110) {  //Funktion F13 bis F20 im Expansion Byte lesen 
        for (int i = 0; i < 8; i++) {
          if (bitRead(decDCCData[2+verschub],i) == 1) {
            Serial.print(" F");
            Serial.print(13+i);          
          }
        }
      }
      if (decDCCData[1+verschub] == B11011111) {  //Funktion F21 bis F28 im Expansion Byte lesen 
        for (int i = 0; i < 8; i++) {
          if (bitRead(decDCCData[2+verschub],i) == 1) {
            Serial.print(" F");
            Serial.print(21+i);          
          }
        }
      }  //ENDE F21 bis F28
*/      
  }
}

//****************************************************************  
//Call this in Setup routine
void DCCDecoder_init() {
  pinMode(decDCCPin, INPUT);    //Dateneingang
  attachInterrupt(0, DCCDecoder_data, RISING);  //ISR für den Dateneingang
  Timer4_initialize(69);   // set a timer4 of length 70 microseconds
}

//****************************************************************  
//Call this in each Loop round
void DCCDecoder_update() {
  if (dataReady == true) {    //Daten vom DCC Decoder   
    dataReady = false;
   DCCDecoder_anaylse();         //eingelesene Daten auswerten.
  }
}

#endif
