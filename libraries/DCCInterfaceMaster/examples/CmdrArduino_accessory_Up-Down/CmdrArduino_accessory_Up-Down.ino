
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
byte prev_state = 1;    //Intellibox red = 0 or green = 1
byte activ_state = 0;
unsigned long timer = 0;
unsigned int address = 4; //this address is not, strictly speaking, the accessory decoder address, but the address as it appears to the user

#define flip_time 1000    //flip time in ms

void setup() {
  Serial.begin(115200);
  dps.setup(6, SwitchFormat);		//DCC Data out PIN and Format
  dps.setpower(true);   //switch power ON

  //set up GO-PIN for Booster
  pinMode(39, OUTPUT);
  digitalWrite(39, LOW); //activate built-in pull-up resistor  
}

void loop() {
  if((millis() - timer) > flip_time) //only do this one per seconds
  {   //switch active - Button goes down
    Serial.print(address);
    if(prev_state)    //get switch state (red/green)?
    {
      dps.setBasicAccessoryPos(address, 1,1); //turn
      Serial.print(" - left");
    }
    else
    {
      dps.setBasicAccessoryPos(address, 0, 1); //staight
      Serial.print(" - right");
    }
    activ_state = true;         //save state - is active now!
    timer = millis();   //save last time
  }
  if(((millis() - timer) > 200) && (activ_state == true)) //only do this after 200ms
  {   //the button goes Up now
    activ_state = false;    //set state inactive
    Serial.println(" - off");
    if(prev_state)
      dps.setBasicAccessoryPos(address, 1,0);
    else
      dps.setBasicAccessoryPos(address, 0,0);
      
    prev_state = !prev_state;   //change state now - we are ready
  }
  
  dps.update(); //do this all the time to update dcc!
}
