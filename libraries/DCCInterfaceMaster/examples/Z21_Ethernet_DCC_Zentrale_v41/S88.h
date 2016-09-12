//--------------------------------------------------------------
/*

  S88 Bus Master Interface
  
*/
#if defined(S88N)

//**************************************************************
//Setup up PIN-Configuration for different MCU
#include "MCU_config.h"

//--------------------------------------------------------------
//Timer select for S88:
#if defined(MEGA_MCU) //Arduino MEGA
#define S88useT3  //S88 mit Timer3

#elif defined(__AVR_ATmega1284P__) //Sanguino (other pins!)
#define S88useT3  //S88 mit Timer3

#else //others Arduino UNO
#define S88noT    //kein Timer oder S88useT2 für Timer2
#endif

//--------------------------------------------------------------
//config:
#define S88MAXMODULE 62

#if defined(S88useT2) //Timer2
//S88 Timer frequency is 250kHz for ( /64 prescale from 16MHz )
#define TIMER2_Time 0x50 //je größer desto schneller die Abfrageintervalle

#elif defined(S88useT3) //Timer3
#define TIMER3_Time 0xBA00

#elif defined(S88noT)   //kein Timer
#define S88CLOCKTIME 400     //Microseconds
unsigned long S88previousMicros = 0;       // will store last time of S88 clock
#endif

unsigned int S88RCount = 0;    //Lesezähler 0-39 Zyklen (S88Module * 8 * 2 + 10)

byte S88MCount = 0;   //Lesezähler für S88 Module
byte S88PCount = 0;   //Lesezähler für S88 Pin am Modul

/*
'0' = keine
 's' = Änderungen vorhanden, noch nicht fertig mit Auslesen
 'i' = Daten vollständig, senden an PC
 */
char S88sendon = '0';        //Bit Änderung
byte S88Module = S88MAXMODULE;    //Anzahl der Module - maximal 62 Module à 16 Ports
byte data[S88MAXMODULE];     //Zustandsspeicher für 62x 8fach Modul

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
  if (S88Module > S88MAXMODULE) { //S88 on!
    S88Module = S88MAXMODULE;   //S88 max Module
    EEPROM.write(EES88Moduls, S88MAXMODULE); //save correct max Value
  }
  if (S88Module > 0) {  //S88 on!
    //Setup Timer?
    #if defined(S88useT2)
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
    TCCR2B = 1<<CS22 | 0<<CS21 | 0<<CS20; //Prescaler
    TIMSK2 = 1<<TOIE2; //Timer2 Overflow Interrupt Enable
    TCNT2=TIMER2_Time; //load the timer for its first cycle
  #elif defined(S88useT3)
    TCCR3A = 0; // Reset control registers timer1 /not needed, safety
    TCCR3B = 0; // Reset control registers timer1 // not needed, safety 
    TCNT3 = TIMER3_Time; //load the timer for its first cycle
    TIMSK3 = 1<<TOIE3; //Timer3 Overflow Interrupt Enable
    TCCR3B = 0<<CS32 | 0<<CS31 | 1<<CS30; //Prescaler
  #endif
  }
}

//--------------------------------------------------------------
//Einlesen des Daten-Bit und Vergleich mit vorherigem Durchlauf
void S88readData() {
  digitalWrite(S88ClkPin, LOW);  //LOW-Flanke, dann liegen die Daten an
  byte getData = digitalRead(S88DataPin);  //Bit einlesen
  if (bitRead(data[S88MCount],S88PCount) != getData) {     //Zustandsänderung Prüfen?
    bitWrite(data[S88MCount],S88PCount,getData);          //Bitzustand Speichern
    S88sendon = 's';  //Änderung vorgenommen. (SET)
    #if defined(DEBUG)
      Debug.print("Sensor ");
      Debug.print(S88MCount*8+S88PCount+1);
      Debug.print("=");
      Debug.println(getData ? "on" : "off");
    #endif  
  }
  S88PCount++;
  if (S88PCount == 8) {
    S88PCount = 0;
    S88MCount++;
  }
}

//-------------------------------------------------------------- 
//S88 power bus
#if defined(S88useT2)
ISR(TIMER2_OVF_vect) {    //Timer2 overflow Interrupt vector handler
#elif defined(S88useT3)
ISR(TIMER3_OVF_vect) {    //Timer2
#elif defined(S88noT)
void getS88Data() {       //kein Timer
#endif  

  if (S88RCount == 3)    //Load/PS Leitung auf 1, darauf folgt ein Schiebetakt nach 10 ticks!
    digitalWrite(S88PSPin, HIGH);
  else if (S88RCount == 4)   //Schiebetakt nach 5 ticks und S88Module > 0
    digitalWrite(S88ClkPin, HIGH);       //1. Impuls
  else if (S88RCount == 5)   //Read Data IN 1. Bit und S88Module > 0
    S88readData();    //LOW-Flanke während Load/PS Schiebetakt, dann liegen die Daten an
  else if (S88RCount == 9)    //Reset-Plus, löscht die den Paralleleingängen vorgeschaltetetn Latches
    digitalWrite(S88ResetPin, HIGH);
  else if (S88RCount == 10)    //Ende Resetimpuls
    digitalWrite(S88ResetPin, LOW);
  else if (S88RCount == 11)    //Ende PS Phase
    digitalWrite(S88PSPin, LOW);
  else if (S88RCount >= 12) {    //Auslesen mit weiteren Schiebetakt der Latches links
    if (S88RCount % 2 == 0)      //wechselnder Taktimpuls/Schiebetakt
      digitalWrite(S88ClkPin, HIGH);  
    else S88readData();    //Read Data IN 2. bis (Module*8) Bit
  }
  S88RCount++;      //Zähler für Durchläufe/Takt
  if (S88MCount == S88Module) {  //Alle Module ausgelesen?
    S88RCount = 0;                    //setzte Zähler zurück
    S88MCount = 0;                  //beginne beim ersten Modul
    S88PCount = 0;                  //beginne beim ersten Port
    //init der Grundpegel
    digitalWrite(S88PSPin, LOW);    
    digitalWrite(S88ClkPin, LOW);
    digitalWrite(S88ResetPin, LOW);
    if (S88sendon == 's')  //Änderung erkannt
      S88sendon = 'i';      //senden
  }
  #if defined(S88useT2)
  //Capture the current timer value. This is how much error we have due to interrupt latency and the work in this function
  TCNT2 = TCNT2 + TIMER2_Time;    //Reload the timer and correct for latency.
  #elif defined(S88useT3)
  TCNT3 = TIMER3_Time; // set initial value to remove time error (16bit counter register)
  #endif
}

//--------------------------------------------------------------------------------------------
void notifyS88Data() {
  if (S88Module != 0) {
    if (S88sendon == 'i' || S88sendon == 'm') {
      byte MAdr = 1;  //Rückmeldemodul
      byte datasend[11];  //Array Gruppenindex (1 Byte) & Rückmelder-Status (10 Byte)
      datasend[0] = 0; //Gruppenindex für Adressen 1 bis 10
      for(byte m = 0; m < S88Module; m++) {  //Durchlaufe alle aktiven Module im Speicher
        datasend[MAdr] = data[m];
        MAdr++;  //Nächste Modul in der Gruppe
        if (MAdr >= 11) {  //10 Module à 8 Ports eingelesen
          MAdr = 1;  //beginne von vorn
          z21.setS88Data (datasend);
          //EthSend (0x0F, 0x80, datasend, false, Z21bcRBus_s); //RMBUS_DATACHANED
          datasend[0]++; //Gruppenindex erhöhen
        }
      }
      if (MAdr < 11) {  //noch unbenutzte Module in der Gruppe vorhanden? Diese 0x00 setzten und dann Melden!
        while (MAdr < 11) {
          datasend[MAdr] = 0x00;  //letzten leeren Befüllen
          MAdr++;   //Nächste Modul in der Gruppe   
        }
        z21.setS88Data (datasend);
        //EthSend (0x0F, 0x80, datasend, false, Z21bcRBus_s); //RMBUS_DATACHANED
      }
      S88sendon = '0';        //Speicher Rücksetzten
    }
    #if defined(S88noT)    
    unsigned long S88Micros = micros();
    if (S88Micros - S88previousMicros > S88CLOCKTIME) {
      getS88Data();    //S88 Bus Takt
      S88previousMicros = S88Micros;
    }
    #endif
  }
}

//-------------------------------------------------------------- 
#endif
//-------------------------------------------------------------- 

