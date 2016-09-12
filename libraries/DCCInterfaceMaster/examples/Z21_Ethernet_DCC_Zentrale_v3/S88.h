//--------------------------------------------------------------
/*

  S88 Bus Master Interface
  
*/
#if defined(S88N)

//--------------------------------------------------------------
//S88 Timer frequency is 250kHz for ( /64 prescale from 16MHz )
#define TIMER_Time 0x50 //je größer desto schneller die Abfrageintervalle
/*
  Der Timer erzeugt den notwendigen Takt für die S88 Schiebeabfragen.
 Je nach verwendten Modulen kann der Takt beliebigt in seiner Geschwindigkeit
 geändert werden, aber nicht jede Hardware unterstützt ein "fast" Auslesen!
 */
//Pinbelegungen am Dekoder:

//Eingänge:
#define S88DataPin A0      //S88 Data IN

//Ausgänge:
#define S88ClkPin A1    //S88 Clock
#define S88PSPin A2    //S88 PS/LOAD
#define S88ResetPin A3    //S88 Reset

byte S88RCount = 0;    //Lesezähler 0-39 Zyklen
byte S88RMCount = 0;   //Lesezähler Modul-Pin

/*
'0' = keine
 's' = Änderungen vorhanden, noch nicht fertig mit Auslesen
 'i' = Daten vollständig, senden an PC
 */
char S88sendon = '0';        //Bit Änderung
byte S88Module = 0;    //Anzahl der Module - maximal 62 Module à 16 Ports
byte data[62];     //Zustandsspeicher für 62x 8fach Modul

//-------------------------------------------------------------- 
//-------------------------------------------------------------- 
void SetupS88() {
  pinMode(S88ResetPin, OUTPUT);    //Reset
  pinMode(S88PSPin, OUTPUT);      //PS/LOAD
  pinMode(S88ClkPin, OUTPUT);      //Clock
  digitalWrite(S88ResetPin, LOW);
  digitalWrite(S88PSPin, LOW);      //init
  digitalWrite(S88ClkPin, LOW);
  pinMode(S88DataPin, INPUT_PULLUP);    //Dateneingang

  S88Module = EEPROM.read(EES88Moduls);
  if (S88Module > 62 || S88Module == 0) { //S88 off!
    S88Module = 0;
    TCCR2B = 0<<CS22 | 0<<CS21 | 0<<CS20;  //Timer 2 off
    return;
  }
  //S88 Aktivieren!

  //Setup Timer2.
  //Configures the 8-Bit Timer2 to generate an interrupt at the specified frequency.
  //Returns the time load value which must be loaded into TCNT2 inside your ISR routine.
  /* 
   16Mhz / 1 prescaler = 16Mhz = CS 001 
   16Mhz / 8 prescaler = 2MHz oder 0,5usec = CS 010
   16Mhz / 64 prescaler = 250kHz = CS 011 
   16Mhz / 256 prescaler = CS 100
   16Mhz / 1024 prescaler = CS 101
   */
  //Timer2 Settings: Timer Prescaler /256
  //Timmer clock = 16MHz/256

  TCCR2A = 0;
  TCCR2B = 1<<CS22 | 0<<CS21 | 0<<CS20;
  TIMSK2 = 1<<TOIE2; //Timer2 Overflow Interrupt Enable
  TCNT2=TIMER_Time; //load the timer for its first cycle
}

//--------------------------------------------------------------
//Einlesen des Daten-Bit und Vergleich mit vorherigem Durchlauf
void S88readData() {
  digitalWrite(S88ClkPin, LOW);  //LOW-Flanke, dann liegen die Daten an
  byte Modul = S88RMCount / 8;
  byte Port = S88RMCount % 8;
  byte getData = digitalRead(S88DataPin);  //Bit einlesen
  if (bitRead(data[Modul],Port) != getData) {     //Zustandsänderung Prüfen?
    bitWrite(data[Modul],Port,getData);          //Bitzustand Speichern
    S88sendon = 's';  //Änderung vorgenommen. (SET)
    #if defined(DEBUG)
      Serial.print("Sensor ");
      Serial.print(Modul*8+Port+1);
      Serial.print("=");
      Serial.println(getData ? "on" : "off");
    #endif  
  }
  S88RMCount++;
}

//-------------------------------------------------------------- 
//Timer ISR Routine
//Timer2 overflow Interrupt vector handler
ISR(TIMER2_OVF_vect) {
//void getS88Data() {
  if (S88RCount == 3)    //Load/PS Leitung auf 1, darauf folgt ein Schiebetakt nach 10 ticks!
    digitalWrite(S88PSPin, HIGH);
  if (S88RCount == 4)   //Schiebetakt nach 5 ticks und S88Module > 0
    digitalWrite(S88ClkPin, HIGH);       //1. Impuls
  if (S88RCount == 5)   //Read Data IN 1. Bit und S88Module > 0
    S88readData();    //LOW-Flanke während Load/PS Schiebetakt, dann liegen die Daten an
  if (S88RCount == 9)    //Reset-Plus, löscht die den Paralleleingängen vorgeschaltetetn Latches
    digitalWrite(S88ResetPin, HIGH);
  if (S88RCount == 10)    //Ende Resetimpuls
    digitalWrite(S88ResetPin, LOW);
  if (S88RCount == 11)    //Ende PS Phase
    digitalWrite(S88PSPin, LOW);
  if (S88RCount >= 12 && S88RCount < 10 + (S88Module * 8) * 2) {    //Auslesen mit weiteren Schiebetakt der Latches links
    if (S88RCount % 2 == 0)      //wechselnder Taktimpuls/Schiebetakt
      digitalWrite(S88ClkPin, HIGH);  
    else S88readData();    //Read Data IN 2. bis (Module*8) Bit
  }
  S88RCount++;      //Zähler für Durchläufe/Takt
  if (S88RCount >= 10 + (S88Module * 8) * 2) {  //Alle Module ausgelesen?
    S88RCount = 0;                    //setzte Zähler zurück
    S88RMCount = 0;                  //beginne beim ersten Modul von neuem
    //init der Grundpegel
    digitalWrite(S88PSPin, LOW);    
    digitalWrite(S88ClkPin, LOW);
    digitalWrite(S88ResetPin, LOW);
    if (S88sendon == 's')  //Änderung erkannt
      S88sendon = 'i';      //senden
  }
  //Capture the current timer value. This is how much error we have due to interrupt latency and the work in this function
  TCNT2 = TCNT2 + TIMER_Time;    //Reload the timer and correct for latency.
}

//--------------------------------------------------------------------------------------------
void notifyS88Data() {
//  if (S88Module != 0)
//    getS88Data();    //S88 Bus Takt
  if (S88sendon == 'i' || S88sendon == 'm') {
    byte MAdr = 1;  //Rückmeldemodul
    byte datasend[11];  //Array Gruppenindex (1 Byte) & Rückmelder-Status (10 Byte)
    datasend[0] = 0; //Gruppenindex für Adressen 1 bis 10
    for(byte m = 0; m < S88Module; m++) {  //Durchlaufe alle aktiven Module im Speicher
      datasend[MAdr] = data[m];
      MAdr++;  //Nächste Modul in der Gruppe
      if (MAdr >= 11) {  //10 Module à 8 Ports eingelesen
        MAdr = 1;  //beginne von vorn
        EthSend (0x0F, 0x80, datasend, false, Z21bcRBus_s); //RMBUS_DATACHANED
        datasend[0]++; //Gruppenindex erhöhen
      }
    }
    if (MAdr < 11) {  //noch unbenutzte Module in der Gruppe vorhanden? Diese 0x00 setzten und dann Melden!
      while (MAdr < 11) {
        datasend[MAdr] = 0x00;  //letzten leeren Befüllen
        MAdr++;   //Nächste Modul in der Gruppe   
      }
      EthSend (0x0F, 0x80, datasend, false, Z21bcRBus_s); //RMBUS_DATACHANED
    }
    S88sendon = '0';        //Speicher Rücksetzten
  }
}

//-------------------------------------------------------------- 
#endif
//-------------------------------------------------------------- 

