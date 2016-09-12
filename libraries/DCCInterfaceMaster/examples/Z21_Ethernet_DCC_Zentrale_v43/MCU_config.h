//--------------------------------------------------------------
/*
 * Setup up PIN-Configuration for different MCU
 * 
 * Support for:
 *    - Arduino UNO
 *    - Arduino MEGA  
 *    - Sanguino (ATmgega 644p & ATmega 1284p)
 * 
 * Copyright (c) by Philipp Gahtow, year 2015
*/

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) //Arduino MEGA
#define MEGA_MCU

#elif defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644P__)  //Sanguino (other pins!)
#define SANGUINO_MCU
//ACHTUNG SS is on PIN3 (D2)!!!

#else //others Arduino UNO
#define UNO_MCU

#endif

//--------------------------------------------------------------
//Z21 Button (Reset & Go/Stop):
#if defined(SANGUINO_MCU)
#define Z21ResetPin 27  //RESET-Button-Pin bei Neustart betätigen um Standard IP zu setzten!
#define Z21ButtonPin Z21ResetPin  //Pin where the POWER-Button is conected

#elif defined(UNO_MCU)
#define Z21ResetPin 2  //RESET-Button-Pin bei Neustart betätigen um Standard IP zu setzten!
#define Z21ButtonPin Z21ResetPin  //Pin where the POWER-Button is conected

#else //other MCU
#define Z21ResetPin 47  //RESET-Button-Pin bei Neustart betätigen um Standard IP zu setzten!
#define Z21ButtonPin Z21ResetPin  //Pin where the POWER-Button is conected
#endif

//--------------------------------------------------------------
//DCC Master & Booster:
#if defined(SANGUINO_MCU)
#define DCCLed 25    //LED to show DCC active
#define DCCPin 12    //Pin for DCC sginal out
#define ShortLed 26     //LED to show Short
#define ShortExtPin 4  //Pin to detect Short Circuit of Booster (detect LOW)
#define GoExtPin 3   //Pin for GO/STOP Signal of Booster
#define KSPin  23   //Pin for using Kehrschleifen-Modul
//Booster INT config:
#define GoIntPin 17   //Pin for second Booster like TLE5205
#define ShortIntPin 13  //Pin for second Booster like TLE5205 (detect HIGH)
#define VAmpIntPin A4   //Input for ACS712t sensor

#else //other MCU
#define DCCLed 3    //LED to show DCC active
#define DCCPin 6    //Pin for DCC sginal out
#define ShortLed 45     //LED to show Short
#define ShortExtPin 5  //Pin to detect Short Circuit of Booster (detect LOW)
#define GoExtPin  A4   //Pin for GO/STOP Signal of Booster
#define KSPin  A5   //Pin for using Kehrschleifen-Modul
//Booster INT config:
#if defined(UNO_MCU)
#define GoIntPin 4   //Pin for second Booster like TLE5205
#define ShortIntPin 2  //Pin for second Booster like TLE5205 (detect HIGH)
#else
#define GoIntPin 39   //Pin for second Booster like TLE5205
#define ShortIntPin 41  //Pin for second Booster like TLE5205 (detect HIGH)
#define VAmpIntPin A9   //Input for ACS712t sensor
#endif

#endif

//--------------------------------------------------------------
//S88 Singel Bus:
#if defined(S88N)
  //Eingänge:
#define S88DataPin A0      //S88 Data IN
  //Ausgänge:
#define S88ClkPin A1      //S88 Clock
#define S88PSPin A2       //S88 PS/LOAD
#define S88ResetPin A3    //S88 Reset
#endif
//--------------------------------------------------------------
//DCC Decoder
#if defined(SANGUINO_MCU)
#undef DECODER
#endif

   //Eingänge:
#define IRQDCCPin 0      //Arduino Interrupt Number (attachInterrupt Funktion)
#define decDCCPin 2      //The Digital PIN where the Interrupt is on

//--------------------------------------------------------------
//XpressNet-Bus:
#if defined(SANGUINO_MCU)
#define XNetTxRxPin  16    //XpressNet Control-Port for Send/Receive at MAX485
  
#else //other MCU
#define XNetTxRxPin  9    //XpressNet Control-Port for Send/Receive at MAX485
#endif

//--------------------------------------------------------------
//LocoNet-Bus:
#if defined(SANGUINO_MCU)
#define LNTxPin 15    //Sending Pin for LocoNet
  
#else //other MCU
#define LNTxPin 7    //Sending Pin for LocoNet
#endif

