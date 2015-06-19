#include <DCC_Decoder.h>
#include <DCCPacket.h>
#include <DCCPacketQueue.h>
#include <DCCPacketScheduler.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Defines and structures
//
#define kDCC_INTERRUPT            1
// This defines Interrupt 0, which is connected to pin 2 on a Arduino UNO
// Other option: interrupt 1, which is connected to pin 3

// CmdrArduino

DCCPacketScheduler dps;
unsigned int analog_value;
char speed_byte, old_speed = 0;
byte count = 0;
byte prev_state = 1;
byte F0 = 0;
const byte DCC_OUT_PIN = 4;

//

typedef struct
{
    int count;
    byte validBytes;
    byte data[6];
} DCCInPacket;

union {
  uint8_t BAR;
  struct {
    uint8_t  r1 : 4; // bit positions 0..3
    uint8_t  r2 : 4; // bit positions 4..7
    // total # of bits just needs to add up to the uint8_t size
  } bar;
} foo;

typedef struct
{
    int address;
    byte locospeed;
    byte light;       // function FL
    byte functions1; // function 1..8
    byte functions2; // function 9..12
    byte functions3; // function 13..20
    byte functions4; // function 21..28
} locos;

const byte MaxNumberOfLocos = 64;
char Answer;
boolean ShowCommands = true;
boolean ShowLocos = true;
boolean ShowDebug = true;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// The dcc decoder object and global data
//
int gPacketCount = 0;
int gIdlePacketCount = 0;
int gLongestPreamble = 0;

DCCInPacket gPackets[25];

locos LocoList[MaxNumberOfLocos] ;
int LocoIndex = 0;

static unsigned long lastMillis = millis();
    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Packet handlers
//

// ALL packets are sent to the RawPacket handler. Returning true indicates that packet was handled. DCC library starts watching for 
// next preamble. Returning false and library continue parsing packet and finds another handler to call.
boolean RawPacket_Handler(byte byteCount, byte* packetBytes)
{
        // Bump global packet count
    ++gPacketCount;
    
    int thisPreamble = DCC.LastPreambleBitCount();
    if( thisPreamble > gLongestPreamble )
    {
        gLongestPreamble = thisPreamble;
    }
    
        // Walk table and look for a matching packet
    for( int i=0; i<(int)(sizeof(gPackets)/sizeof(gPackets[0])); ++i )
    {
        if( gPackets[i].validBytes )
        {
                // Not an empty slot. Does this slot match this packet? If so, bump count.
            if( gPackets[i].validBytes==byteCount )
            {
                char isPacket = true;
                for( int j=0; j<byteCount; j++)
                {
                    if( gPackets[i].data[j] != packetBytes[j] )
                    {
                        isPacket = false;
                        break;
                    } 
                }
                if( isPacket )
                {
                   gPackets[i].count++;
                   return false;
                }
            }
        }else{
                // Empty slot, just copy over data
            gPackets[i].count++;
            gPackets[i].validBytes = byteCount;
            for( int j=0; j<byteCount; j++)
            {
                gPackets[i].data[j] = packetBytes[j];
            }
            return false;
        }
    }    
    
    return false;
}

// Idle packets are sent here (unless handled in rawpacket handler). 
void IdlePacket_Handler(byte byteCount, byte* packetBytes)
{
    ++gIdlePacketCount;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Setup
//
void setup() 
{ 
   Serial.begin(115200);
   
// CmdrArduino setup
   dps.setup();
//  

   DCC.SetRawPacketHandler(RawPacket_Handler);   
   DCC.SetIdlePacketHandler(IdlePacket_Handler);  
   DCC.SetupMonitor( kDCC_INTERRUPT );   
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CollectTable()
{
    char buffer60Bytes[60];
    
    if (Answer == 's') {
      DumpStats();
    }
    
    for( int i=0; i<(int)(sizeof(gPackets)/sizeof(gPackets[0])); ++i )
    {
        if( gPackets[i].validBytes > 0 )
        {
            if (ShowCommands) {
              Serial.print(gPackets[i].count, DEC);
              if( gPackets[i].count < 10 )
              {
                  Serial.print("        ");
              }else{
                  if( gPackets[i].count < 100 )
                  {
                      Serial.print("       ");
                  }else{
                      Serial.print("      ");
                  }
              }
            }
            
            int IntSpeed;
            int Address = gPackets[i].data[0];
            int Valid = 0;
            byte Instruction;
            byte Databyte;
            if (Address < 128) 
            {
              Instruction = gPackets[i].data[1];
              Databyte = gPackets[i].data[2];
              Valid = 1;
            }
            else
            {
              if ( ((Address & 0xC0) == 0xC0) && ((Address & 0x3F) != 0x3F))
              {
                Address = (Address & 0x1F) * 256 + gPackets[i].data[1];
                Instruction = gPackets[i].data[2];
                Databyte = gPackets[i].data[3];
                Valid = 1;
              }
            }
            if (Valid) {
              if (ShowCommands) {
                Serial.print ("NMRA DCC Packet: Loc ");
                Serial.print (Address);
              }
              int wantedpos = -1;
              for (int i = 0; i < MaxNumberOfLocos; i++) {
                if (Address == LocoList[i].address) {
                  wantedpos = i;
                  break;
                }
              }
              if (wantedpos == -1) 
              {
                LocoIndex += 1;
                LocoList[LocoIndex].address = Address;
                LocoList[LocoIndex].functions3 = 0x00;
                LocoList[LocoIndex].functions4 = 0x00;
                wantedpos = LocoIndex;
                int locosort = wantedpos;
                while ((LocoList[locosort].address < LocoList[locosort -1].address) && locosort > 0)
                {
                  locos tempadress = LocoList[locosort];
                  LocoList[locosort] = LocoList[locosort-1];
                  LocoList[locosort-1] = tempadress;
                  locosort -= 1;
                }
              }
              int InstructionType = (Instruction & 0xE0) >> 5;
              IntSpeed = ((Instruction & 0x0F) << 1) + ((Instruction & 0x10) >> 4);
              if ( IntSpeed == 1 ) 
              { 
                IntSpeed = 0; 
              }
              else 
              { 
                if ( IntSpeed == 2 ) 
                { 
                  IntSpeed = 0; 
                }
                else
                {
                  if ( IntSpeed == 3 ) 
                  { 
                    IntSpeed = 0; 
                  }
                  else
                  {
                    IntSpeed -= 3;
                  }
                }  
              }
              switch (InstructionType) {
                case B000:
                  if (ShowCommands) {
                    Serial.print(F(" Consist | "));
                  }
                  break;
                case B001:
                  if (ShowCommands) {
                    Serial.print(F(" Advanced | "));
                  }
                  break;
                case B010:
                  if (ShowCommands) {
                    Serial.print(F(" Reverse, Speed: "));
                    Serial.print(IntSpeed);
                  }
                  LocoList[wantedpos].locospeed = IntSpeed;
                  LocoList[wantedpos].locospeed += 0x80;
                  break;               
                case B011:
                  if (ShowCommands) {
                    Serial.print(F(" Forward, Speed: "));
                    Serial.print(IntSpeed);
                  }
                  LocoList[wantedpos].locospeed = IntSpeed;
                  break;
                case B100:
                  if (ShowCommands) {
                    Serial.print(F(" Function FL, F1..F4: light is "));
                  }
                  if ( Instruction & 0x10 ) 
                  { 
                    if (ShowCommands) {
                      Serial.print("on, ");
                    }
                    LocoList[wantedpos].light = 1;
                  }
                  else
                  {
                    if (ShowCommands) {
                      Serial.print("off, ");
                    }
                    LocoList[wantedpos].light = 0;
                  }
                  if (ShowCommands) {
                    Serial.print( Instruction & 0x0F , BIN);
                  }
                  foo.bar.r2 = (Instruction & 0x0F);
                  LocoList[wantedpos].functions1 = foo.BAR;
                  break;
                case B101:
                  if ( Instruction & 0x10 )
                  {
                    if (ShowCommands) {
                      Serial.print(F(" Function F5..F8: "));
                    }
                    foo.bar.r1 = (Instruction & 0x0F);
                    LocoList[wantedpos].functions1 = foo.BAR;
                  }
                  else
                  {
                    if (ShowCommands) {
                      Serial.print(" Function F9..F12: ");
                    }
                  }
                  if (ShowCommands) {
                    Serial.print( Instruction & 0x0F , BIN);
                  }
                  break;
                case B110:
                  if ( (Instruction & 0x1E) == 0x1E ) 
                  {
                    if (ShowCommands) {
                      Serial.print(F(" Function F13..F20: "));
                      Serial.print( Databyte ,BIN);
                    }
                    LocoList[wantedpos].functions3 = Databyte;
                  }
                  if ( (Instruction & 0x1F) == 0x1F ) 
                  {
                    if (ShowCommands) {
                      Serial.print(F(" Function F21..F28: "));
                      Serial.print( Databyte ,BIN);
                    }
                    LocoList[wantedpos].functions4 = Databyte;
                  }                  
                  break;
              }
              // Serial.println();
              if (ShowDebug)
              {
                if (ShowCommands) {
                  Serial.print(" | ");
                  Serial.println( DCC.MakePacketString(buffer60Bytes, gPackets[i].validBytes, &gPackets[i].data[0]) );
                }
              }
              else
              {
                if (ShowCommands) {
                  Serial.println();
                }
              }
            }
            else {
              if (ShowCommands) {
                Serial.print("Other data: ");
                Serial.println( DCC.MakePacketString(buffer60Bytes, gPackets[i].validBytes, &gPackets[i].data[0]) );
              }
            }
        }
        gPackets[i].validBytes = 0;
        gPackets[i].count = 0;
    }
    if (Serial.available()) 
    {
      Answer = Serial.read();
      switch (Answer) {
        case 'l':
          ShowLocos ^= 0x01;
          if (ShowLocos) {
            Serial.println(F("List of Locos shown"));
          }
          else
          {
            Serial.println(F("List of Locos hidden"));
            Serial.println(F("====================(l for loclist, d for debug, s for statistics, m to toggle monitor)========================"));
          } 
        case 'm':
          ShowCommands ^= 0x01;
          if (ShowCommands) {
            Serial.println(F("DCC Commands shown"));
          }
          else
          {
            Serial.println(F("DCC Commands hidden"));
            Serial.println(F("====================(l for loclist, d for debug, s for statistics, m to toggle monitor)========================"));
          } 
          break;
        case 'd':
          ShowDebug ^= 0x01;
          if (ShowDebug) {
            Serial.println(F("Debug mode on"));
          }
          else
          {
            Serial.println(F("Debug mode off"));
            Serial.println(F("====================(l for loclist, d for debug, s for statistics, m to toggle monitor)========================"));
          }           
        default:    
          Serial.println(F("====================(l for loclist, d for debug, s for statistics, m to toggle monitor)========================"));
      }
    }
    gPacketCount = 0;
    gIdlePacketCount = 0;
    gLongestPreamble = 0;
}

void DumpStats() {
    Serial.print(F("Total Packet Count: "));
    Serial.println(gPacketCount, DEC);
    Serial.print(F("Idle Packet Count:  "));
    Serial.println(gIdlePacketCount, DEC);
    Serial.print(F("Longest Preamble:  "));
    Serial.println(gLongestPreamble, DEC);
    Serial.println(F("Aantal    Packet_Data"));
}

void DumpTable() {
    Serial.println(F("============================================"));
    Serial.println("Loc listing");
    if (LocoList[1].address == 0)
    {
      Serial.println(F("No locs found or no DCC signal"));
    }
    else
    {
      Serial.println(F("Address / Speed / Light / F1..8 / F9..12 / F13..F20 / F21..28"));
    for (int i = 0; i < MaxNumberOfLocos; i = i + 1) {
      if (LocoList[i].address != 0) {
        Serial.print(i);
        Serial.print(":");
        Serial.print(LocoList[i].address);
        Serial.print(" / ");
        if ((LocoList[i].locospeed & 0x80) == 0x80)
        {
          Serial.print("Rev ");
          Serial.print(LocoList[i].locospeed & 0x7F);
        }
        else
        {
          Serial.print("Fwd ");
          Serial.print(LocoList[i].locospeed);
        }
        Serial.print(" / ");
        Serial.print(LocoList[i].light,BIN);
        Serial.print(" / ");
        Serial.print(LocoList[i].functions1,BIN);
        Serial.print(" / ");
        Serial.print(LocoList[i].functions2,BIN);
        Serial.print(" / ");
        Serial.print(LocoList[i].functions3,BIN);
        Serial.print(" / ");
        Serial.println(LocoList[i].functions4,BIN);      }
     }
    }
    Answer = ' ';
}

//

void GenerateLocoCommands() {
  for (int i = 0; i < MaxNumberOfLocos; i = i + 1) {
    if (LocoList[i].address != 0) {
      if ((LocoList[i].address) < 128) 
      {
        dps.setSpeed28(LocoList[i].address,DCC_SHORT_ADDRESS,1);
//        dps.update();
        dps.setFunctions(LocoList[i].address,DCC_SHORT_ADDRESS,LocoList[i].functions1);
//        dps.update();
      }
      else
      {
        dps.setSpeed28(LocoList[i].address,DCC_LONG_ADDRESS,1);
//        dps.update();
        dps.setFunctions(LocoList[i].address,DCC_LONG_ADDRESS,LocoList[i].functions1);
//        dps.update();
     }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Main loop
//
void loop()
{
    DCC.loop();
    
// CmdrArduino    
//   GenerateLocoCommands();
   dps.update();
    ++count;
//

    if( millis()-lastMillis > 3000 )
    {
        Serial.println(count);
        CollectTable();
        if (ShowLocos)
        {
           DumpTable();
        }
        lastMillis = millis();
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

