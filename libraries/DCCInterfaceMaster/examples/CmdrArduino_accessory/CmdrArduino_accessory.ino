
/********************
* modified by Philipp Gahtow 2015 digitalmoba@arcor.de
*
* Creates a minimum DCC command station that flips a turnout open and closed.
* The DCC waveform is output on Pin define in Setup, and is suitable for connection to an LMD18200-based booster directly,
* or to a single-ended-to-differential driver, to connect with most other kinds of boosters.
********************/

#include <DCCPacketScheduler.h>
#define SwitchFormat IB   //ROCO (+4) or IB (+0)

DCCPacketScheduler dps;
byte prev_state = 1;
unsigned long timer = 0;
unsigned int address = 4; //this address is not, strictly speaking, the accessory decoder address, but the address as it appears to the user

void setup() {
  Serial.begin(115200);
  dps.setup(6, SwitchFormat);		//DCC Data out PIN
  dps.setpower(true);

  //set up button on pin 4
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH); //activate built-in pull-up resistor  
}

void loop() {
  if(millis() - timer > 1000) //only do this one per seconds
  {
    Serial.print(address);
    if(prev_state)
    {
      dps.setBasicAccessoryPos(address, 1);
      Serial.println(" - on");
    }
    else
    {
      dps.setBasicAccessoryPos(address, 0);
      Serial.println(" - off");
    }
    prev_state = prev_state?0:1;
    timer = millis();
  }
  
  dps.update();
}
