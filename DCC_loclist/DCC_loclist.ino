#include <DCC_Decoder.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Defines and structures
//
#define kDCC_INTERRUPT            0

typedef struct
{
    int count;
    byte validBytes;
    byte data[6];
} DCCPacket;

typedef struct
{
    int address;
    int locospeed;
    int light;
    byte functions1;
    byte functions2;
    byte functions3;
} locos;

const byte MaxNumberOfLocos = 64;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// The dcc decoder object and global data
//
int gPacketCount = 0;
int gIdlePacketCount = 0;
int gLongestPreamble = 0;

DCCPacket gPackets[25];

locos LocoList[MaxNumberOfLocos];
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
    
   DCC.SetRawPacketHandler(RawPacket_Handler);   
   DCC.SetIdlePacketHandler(IdlePacket_Handler);
            
   DCC.SetupMonitor( kDCC_INTERRUPT );   
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void DumpAndResetTable()
{
    char buffer60Bytes[60];
    
    for( int i=0; i<(int)(sizeof(gPackets)/sizeof(gPackets[0])); ++i )
    {
        if( gPackets[i].validBytes > 0 )
        {
            int IntSpeed;
            int Address = gPackets[i].data[0];
            int Valid = 0;
            byte Instruction;
            if (Address < 128) 
            {
              Instruction = gPackets[i].data[1];
              Valid = 1;
            }
            else
            {
              if ( ((Address & 0xC0) == 0xC0) && ((Address & 0x3F) != 0x3F))
              {
                Address = (Address & 0x1F) * 256 + gPackets[i].data[1];
                Instruction = gPackets[i].data[2];
                Valid = 1;
              }
            }
            if (Valid) {
//              Serial.print ("NMRA DCC Packet: Loc ");
//              Serial.print (Address);
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
//                  Serial.print(" Consist | ");
                  break;
                case B001:
//                  Serial.print(" Advanced | ");
                  break;
                case B010:
//                  Serial.print(" Reverse, Speed: ");
//                  Serial.print(IntSpeed);
                  LocoList[wantedpos].locospeed = -1 * IntSpeed;
                  break;               
                case B011:
//                  Serial.print(" Forward, Speed: ");
//                  Serial.print(IntSpeed);
                  LocoList[wantedpos].locospeed = IntSpeed;
                  break;
                case B100:
//                  Serial.print(" Function FL, F1..F4: light is ");
                  if ( Instruction & 0x10 ) 
                  { 
//                    Serial.print("on, ");
                      LocoList[wantedpos].light = 1;
                  }
                  else
                  {
//                    Serial.print("off, ");
                      LocoList[wantedpos].light = 0;
                  }
//                  Serial.print( Instruction & 0x0F );
                    LocoList[wantedpos].functions1 = LocoList[wantedpos].functions1 | (Instruction & 0x0F);
                  break;
                case B101:
                  if ( Instruction & 0x10 )
                  {
//                    Serial.print(" Function F5..F8: ");
                      LocoList[wantedpos].functions1 = LocoList[wantedpos].functions1 | ((Instruction & 0x0F) << 4);
                  }
                  else
                  {
//                    Serial.print(" Function F9..F12: ");
                      LocoList[wantedpos].functions2 = LocoList[wantedpos].functions2 | (Instruction & 0x0F);
                  }
//                  Serial.print( Instruction & 0x0F );
                  break;
                case B110:
                  if ( (Instruction & 0x1E) == 0x1E ) 
                  {
//                    Serial.print(" Function F13..F20: ");
//                    Serial.print( Instruction & 0x0F );
                      LocoList[wantedpos].functions3 = gPackets[i].data[3];
                  }
                  break;
              }
              Serial.println();
              // Serial.print(" | ");
              // Serial.println( DCC.MakePacketString(buffer60Bytes, gPackets[i].validBytes, &gPackets[i].data[0]) );
            }
            else {
              Serial.print("Other data: ");
              Serial.println( DCC.MakePacketString(buffer60Bytes, gPackets[i].validBytes, &gPackets[i].data[0]) );
            }
        }
        gPackets[i].validBytes = 0;
        gPackets[i].count = 0;
    }
    Serial.println("============================================");
    Serial.println("Loc listing");
    if (LocoList[1].address == 0)
    {
      Serial.println("No locs found or no DCC signal");
    }
    else
    {
      Serial.println("Address / Speed / Light / F1..8 / F9..16 / F17..20");
    for (int i = 0; i < MaxNumberOfLocos; i = i + 1) {
      if (LocoList[i].address != 0) {
        Serial.print(LocoList[i].address);
        Serial.print(" / ");
        Serial.print(LocoList[i].locospeed);
        Serial.print(" / ");
        Serial.print(LocoList[i].light,BIN);
        Serial.print(" / ");
        Serial.print(LocoList[i].functions1,BIN);
        Serial.print(" / ");
        Serial.print(LocoList[i].functions2,BIN);
        Serial.print(" / ");
        Serial.println(LocoList[i].functions3,BIN);      }
     }
    Serial.println("============================================");
    }

    gPacketCount = 0;
    gIdlePacketCount = 0;
    gLongestPreamble = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Main loop
//
void loop()
{
    DCC.loop();
    
    if( millis()-lastMillis > 1000 )
    {
        DumpAndResetTable();
        lastMillis = millis();
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

