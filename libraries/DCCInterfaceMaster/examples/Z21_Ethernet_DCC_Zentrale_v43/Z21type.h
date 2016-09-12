/*
 * Z21type.h
 * Created on: 16.04.2015
 *
 * Copyright (c) by Philipp Gahtow, year 2015
*/
//**************************************************************
//Z21 LAN Protokoll V1.05 (21.01.2015)
#define z21Port 21105  //local port to listen Z21-Protokoll on UDP
//Firmware-Version der Z21:
#define FWVersionMSB 0x01
#define FWVersionLSB 0x26
//Hardware-Typ der Z21: 0x00000201 // Z21 (Hardware-Variante ab 2013)
#define HWTypeMSB 0x02
#define HWTypeLSB 0x01

//**************************************************************
//Client Configuration:

//Speichergröße:
#if defined(__AVR_ATmega1284P__)
#define LANmaxIP 40     //max IP-Adressen (max Clients)
#elif defined(MEGA_MCU)  //Arduino MEGA
#define LANmaxIP 24
#elif defined (__AVR_ATmega644P__)
#define LANmaxIP 12
#else   //Arduino UNO
#define LANmaxIP 5
#endif
//Serialport:
#ifndef WLAN
#if defined(MEGA_MCU) //Arduino MEGA
#define WLAN Serial2
#else
#define WLAN Serial
#endif
#endif

#define WLANmaxIP 30    //Anzahl Clients über ESP

#define ActTimeIP 20    //Aktivhaltung einer IP für (sec./2)
#define interval 2000   //interval in milliseconds for checking IP aktiv state

//**************************************************************
//Z21 LAN Protokoll Spezifikation:
#define LAN_X_Header                 0x40  //not in Spezifikation!
#define LAN_GET_SERIAL_NUMBER        0x10
#define LAN_LOGOFF                   0x30
#define LAN_X_GET_SETTING            0x21  
#define LAN_X_BC_TRACK_POWER         0x61
#define LAN_X_UNKNOWN_COMMAND        0x61
#define LAN_X_STATUS_CHANGED         0x62
#define LAN_X_SET_STOP               0x80  //AW: LAN_X_BC_STOPPED
#define LAN_X_BC_STOPPED             0x81
#define LAN_X_GET_FIRMWARE_VERSION   0xF1  //AW: 0xF3
#define LAN_SET_BROADCASTFLAGS       0x50
#define LAN_GET_BROADCASTFLAGS       0x51
#define LAN_SYSTEMSTATE_DATACHANGED  0x84
#define LAN_SYSTEMSTATE_GETDATA      0x85  //AW: LAN_SYSTEMSTATE_DATACHANGED
#define LAN_GET_HWINFO               0x1A
#define LAN_GET_LOCOMODE             0x60
#define LAN_SET_LOCOMODE             0x61
#define LAN_GET_TURNOUTMODE          0x70
#define LAN_SET_TURNOUTMODE          0x71
#define LAN_X_GET_LOCO_INFO          0xE3
#define LAN_X_SET_LOCO               0xE4  //X-Header
#define LAN_X_SET_LOCO_FUNCTION      0xF8  //DB0
#define LAN_X_LOCO_INFO              0xEF
#define LAN_X_GET_TURNOUT_INFO       0x43 
#define LAN_X_SET_TURNOUT            0x53
#define LAN_X_TURNOUT_INFO           0x43 
#define LAN_X_CV_READ                0x23
#define LAN_X_CV_WRITE               0x24
#define LAN_X_CV_NACK_SC             0x61
#define LAN_X_CV_NACK                0x61
#define LAN_X_CV_RESULT              0x64
#define LAN_RMBUS_DATACHANGED        0x80
#define LAN_RMBUS_GETDATA            0x81
#define LAN_RMBUS_PROGRAMMODULE      0x82

#define LAN_RAILCOM_DATACHANGED      0x88
#define LAN_RAILCOM_GETDATA          0x89

#define LAN_LOCONET_Z21_RX           0xA0
#define LAN_LOCONET_Z21_TX           0xA1
#define LAN_LOCONET_FROM_LAN         0xA2
#define LAN_LOCONET_DISPATCH_ADDR    0xA3
#define LAN_LOCONET_DETECTOR         0xA4

#define LAN_X_CV_POM                 0xE6  //X-Header 

//ab Z21 FW Version 1.23
#define LAN_X_MM_WRITE_BYTE          0x24

//ab Z21 FW Version 1.25
#define LAN_X_DCC_READ_REGISTER      0x22
#define LAN_X_DCC_WRITE_REGISTER     0x23

//**************************************************************
//Z21 BC Flags
#define Z21bcNone                B00000000
#define Z21bcAll             0x00000001
#define Z21bcAll_s               B00000001
#define Z21bcRBus            0x00000002
#define Z21bcRBus_s              B00000010
//#define Z21bcRailcom         0x00000004    //zukünftige Erweiterung (siehe Z21 Protokoll V1.05)
#define Z21bcSystemInfo      0x00000100
#define Z21bcSystemInfo_s        B00000100

#define Z21bcNetAll          0x00010000 // Alles, auch alle Loks ohne vorher die Lokadresse abonnieren zu müssen (für PC Steuerung)
#define Z21bcNetAll_s            B00001000

#define Z21bcLocoNet         0x01000000 // LocoNet Meldungen an LAN Client weiterleiten (ohne Loks und Weichen)
#define Z21bcLocoNet_s           B00010000
#define Z21bcLocoNetLocos    0x02000000 // Lok-spezifische LocoNet Meldungen an LAN Client weiterleiten
#define Z21bcLocoNetLocos_s      B00100000
#define Z21bcLocoNetSwitches 0x04000000 // Weichen-spezifische LocoNet Meldungen an LAN Client weiterleiten
#define Z21bcLocoNetSwitches_s   B01000000
#define Z21bcLocoNetGBM      0x08000000  //Status-Meldungen von Gleisbesetztmeldern am LocoNet-Bus
#define Z21bcLocoNetGBM_s        B10000000

//-----------------------------------------
