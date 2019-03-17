//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                          //
//                                 Mantel Clock With Chimes And Daylight Savings                            //
//                                                                                                          //
//                                 Copyright (c) 2018 By Zumwalt Properties, LLC.                           //
//                                                                                                          //
//                                             All Rights Reserved.                                         //
//                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//
// Purchased components.
//
//  1) Feather esp32                : https://www.adafruit.com/product/3405
//  2) Feather Music Maker          : https://www.adafruit.com/product/3436
//  3) Speakers                     : https://www.amazon.com/gp/product/B01LN8ONG4/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1
//  4) Stepper motor and controller : https://www.amazon.com/gp/product/B077YGWRHK/ref=oh_aui_detailpage_o01_s00?ie=UTF8&psc=1

//
// Change log.
//
//
// 2018/12/03, Version 01.00.00.
//
//  1) Initial build.
//

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// User settings.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Compiler.
    
    #define         HOME_SWITCH_CALIBRATE false             // home switch calibrate (true) or not (false)
    #define         USE_SERIAL_PRINTF     false             // use serial.printf (true) or not (false)

    // Daylight savings time.
    
    const int             nDstEndMonth    = 11;             // end month of year (1 == January, 0 for dst off)
    const int             nDstEndDay      = 0;              // 0 for end on first Sunday, 1 through 28-31 for specific date
    const int             nDstEndHour     = 2;              // end hour (0 through 23)
    
    const int             nDstStartMonth  = 3;              // start month of year (1 == January, 0 for dst off)
    const int             nDstStartDay    = 0;              // 0 for start on second Sunday, 1 through 28-31 for specific date
    const int             nDstStartHour   = 2;              // start hour (0 through 23)
    
    // Time zone.
    
    const unsigned long   nTimeZone       = -6;             // offset from utc to your timezone (code was designed in Oklahoma, US central time zone)

    // Wifi.
    
    const char            chPassword[]    = "yourpassword"; // your network password
    const char            chSSID[]        = "yourssid";     // your network SSID
    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Includes.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

    #include <SD.h>                                         // for sd card
    #include <SPI.h>                                        // for spi
    #include <WiFi.h>                                       // for wifi

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Constants.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
  // "Adafruit Feather Music Maker".

    // Communication.
    
      #define VS1053_DATA_XFER_SIZE               32          // vs1053 maximum data transfer size
      
      #define VS1053_SCI_READ                     0x03        // vs1053 read
      #define VS1053_SCI_WRITE                    0x02        // vs1053 write
      
      #define VS1053_REG_MODE                     0x00        // vs1053 address of mode register
      #define VS1053_REG_STATUS                   0x01        // vs1053 address of status register
      #define VS1053_REG_CLOCKF                   0x03        // vs1053 address of clock frequency register
      #define VS1053_REG_VOLUME                   0x0B        // vs1053 address of volume register
      
      #define VS1053_MODE_SM_RESET                0x0004      // vs1053 reset command
      #define VS1053_MODE_SM_SDINEW               0x0800      // vs1053 sdi new command
  
    // Pins.
    
      #define SCK                                 5           // spi clock pin number
      #define MISO                                19          // spi master input slave output pin number
      #define MOSI                                18          // spi master output slave input pin number
      
      #define SD_CS                               14          // sd card chip select pin number
      #define VS1053_CS                           32          // vs1053 chip select pin number
      #define VS1053_DCS                          33          // vs1053 data/command select pin pin number
      #define VS1053_DREQ                         15          // vs1053 data request pin number

  // Clock structure.
  //
  // A modified version of the tm structure.
  
  struct tmClock
  {
    // tm structure styled variables.
    
    int           tm_sec;                                   // 0 through 59
    int           tm_min;                                   // 0 through 59
    int           tm_hour;                                  // 0 through 23
    int           tm_mday;                                  // 1 through 28 - 31
    int           tm_mon;                                   // 0 through 11
    int           tm_year;                                  // 1970 through 2036ish
    int           tm_wday;                                  // 0 (Sunday) through 6 (Saturday)
    int           tm_yday;                                  // not currently implemented
    int           tm_isdst;                                 // 1 if daylight savings time is on, 0 if not
    
    // Modifications.
    
    unsigned long ulTime;                                   // time in seconds since 1970
    unsigned long ulDstStart;                               // daylight savings time start in seconds since 1970
    unsigned long ulDstEnd;                                 // daylight savings time end in seconds since 1970
    bool          bDstValid;                                // daylight savings time start and end is valid (true) or not (false)
  };
  
  // Motor.
    
    #define MOTOR_STEP_FAST                     1           // fast step speed
    #define MOTOR_STEP_NORMAL                   4           // normal step speed
    #define MOTOR_STEPS_PER_HOUR                4096        // steps for one full revolution

  // Ntp.

    enum    NtpPacketMode {REQUEST, SENT, RECEIVED};        // possible values for an NtpPacketMode variable.
    #define NTP_PACKET_LENGTH                   48          // size, in bytes, of an ntp packet
    #define PACKET_DELAY_RESET                  5           // delay (in seconds at 1hz loop()) before resending an ntp packet request
  
  // Pins.
  
    #define LED_PIN                             13          // on board led drive pin number
    #define MOTOR_PHASE_A                       A0          // motor phase a pin number
    #define MOTOR_PHASE_B                       A1          // motor phase b pin number
    #define MOTOR_PHASE_C                       A5          // motor phase c pin number
    #define MOTOR_PHASE_D                       21          // motor phase d pin number
    #define SWITCH_PIN                          27          // 12:00 switch input pin number
   
  // Serial_printf() control.

  #if USE_SERIAL_PRINTF
      #define Serial_printf                     Serial.printf
  #else
      #define Serial_printf                     //
  #endif

  // Time.

    #define SECONDS_PER_MINUTE                  60UL
    #define SECONDS_PER_HOUR                    (SECONDS_PER_MINUTE * 60UL)
    #define SECONDS_PER_DAY                     (SECONDS_PER_HOUR * 24UL)
    
  // UDP.

    #define UDP_PORT                            4000        // ntp time server udp port

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Global Variables.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Clock.

    bool          bClockHandsUpdated            = false;  // clock hands have been updated (true) or not (false)
    bool          bClockStatsValid              = false;  // clock statistics are valid (true) or not (false)
    
  // Days of the week.
    
    const  char*  chDaysOfWeek[]                 = {
                                                      "Sun",
                                                      "Mon",
                                                      "Tue",
                                                      "Wed",
                                                      "Thu",
                                                      "Fri",
                                                      "Sat"
                                                   };

    // Interrupts.

    hw_timer_t *                timerLoop             = NULL;
    volatile SemaphoreHandle_t  timerLoopSemaphore;

    // Months of the year.
    
    const char*   chMonth[]                       = {
                                                      "Jan",
                                                      "Feb",
                                                      "Mar",
                                                      "Apr",
                                                      "May",
                                                      "Jun",
                                                      "Jul",
                                                      "Aug",
                                                      "Sep",
                                                      "Oct",
                                                      "Nov",
                                                      "Dec"
                                                    };

  // mp3.

    File            fMp3Current;                            // mp3 file pointer
    char            chMp3CurrentName[81];                   // mp3 file name
    unsigned long   nMp3CurrentPosition           = 0;      // mp3 file position   
    unsigned long   nMp3CurrentSize               = 0;      // mp3 file size
    int             nMp3CurrentVolume             = 10;     // mp3 playback volume
    bool            bMp3PlackEnabled              = false;  // play mp3 (true) or not (false)
    bool            bMp3Playing                   = false;  // mp3 is playing is playing (true) or not (false)
    
  // ntp.

    byte            chNtpPacket[NTP_PACKET_LENGTH];         // ntp packet
  
  // Stats.
  
    unsigned long ulMicrosAverage                 = 0;
    unsigned long ulMicrosCurrent                 = 0;
    unsigned long ulMicrosHigh                    = 0;
    unsigned long ulMicrosLow                     = 4294967295UL;
  
  // Stepper motor.

    int             nMillisecondsStepHold         = MOTOR_STEP_FAST;    // time in milliseconds between motor steps
    bool            bStepperInUse                 = false;              // stepper motor in use (true) or not (false)
    
  // Time.

    struct tmClock  tmTime = {0, 0, 0, 1, 2, 2018, 0, 0, 0, 0, 0, false}; // where time occurs
    bool            bTimeStatsValid                = false;
   
  // Udp.
    
    WiFiUDP         Udp;                                    // udp structure

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Interrupts.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//
// onTimer
// Called by interrupt service.
// Entry   : nothing
// Returns : nothing
// Notes   :
//
//  1) Via xSemaphoreGiveFromISR(), enable the next loop() pass.
//

void IRAM_ATTR onTimer()
{
  // Give semaphore.
  
  xSemaphoreGiveFromISR(timerLoopSemaphore, NULL);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Tasks.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//
// taskClockHome
// Task to home the clock to the 12:00 position.
// Entry   : pointer to tmClock structure
// Returns : nothing
//

void  taskClockHome(void * tmTime)
{
  // Check stepper motor availability.
  
  if(bStepperInUse)
  {
    // Debug.
    
    Serial_printf("taskHome(): stepper motor in use, task terminated.\n");
                   
    // Stepper motor in use, end this task.
    
    vTaskDelete(NULL);  
  }
  
  // Stepper motor not in use,  but it is now.

  bStepperInUse = true;

  // Debug.
  
  Serial_printf("taskHome(): homing clock to 12:00.\n");

  // Turn on the on board led.
  
  digitalWrite(LED_PIN, HIGH);

  // Speed up.

  nMillisecondsStepHold = MOTOR_STEP_FAST;
  
  // Move off of the 12:00 switch.
  
  while(!digitalRead(SWITCH_PIN))
  {
    Step(1);
  }
  digitalWrite(LED_PIN, LOW);

  // Move onto the 12:00 switch.
  
  while(digitalRead(SWITCH_PIN))
  {
    Step(1);
  }

  // Turn off the on board led.
  
  digitalWrite(LED_PIN, LOW);
  
  // Remove motor power.

  MotorOff();

  // Resume normal speed.

  nMillisecondsStepHold = MOTOR_STEP_NORMAL;

  // Clock is homed.
  
    // Debug.
    
    Serial_printf("taskHome(): clock homed at 12:00.\n");

    // Hour, minutes and seconds to zero.

    ((tmClock *)tmTime)->tm_hour = ((tmClock *)tmTime)->tm_min = ((tmClock *)tmTime)->tm_sec = 0;
    DateAndTimeToSeconds((tmClock *)tmTime);

  // Stepper no longer in use.

  bStepperInUse = false;
  
  // End of task.

  vTaskDelete(NULL);
}

//
// taskMp3Play
// Task to perform sd card mp3 playbock on the vs1053 via spi.
// Entry   : pointer to arguments
// Returns : nothing
//

void  taskMp3Play(void * chMp3Name)
{
  // mp3 is playing.

  bMp3Playing = true;

  // Make a local copy of the filename.

  char  chMp3NameLocal[81];
  strcpy(chMp3NameLocal, (char *)chMp3Name);
  
  // Check for play enabled.
  
  while(bMp3PlackEnabled)
  {
      // Play is enabled, check the vs1053.

      if(digitalRead(VS1053_DREQ))
      {
      // vs1053 is ready for more data, check for availability of more mp3 data.

        if(fMp3Current.available())
        {
          // More mp3 data is available, read mp3 data from sd card.

          unsigned char chBuffer[VS1053_DATA_XFER_SIZE + 1];
          int nByteCount = fMp3Current.read(chBuffer, VS1053_DATA_XFER_SIZE);

          // Send mp3 data to the vs1053.
          
          SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
          digitalWrite(VS1053_DCS, LOW);
          SPI.writeBytes(chBuffer, nByteCount);
          digitalWrite(VS1053_DCS, HIGH);
          SPI.endTransaction();

          // Update nMp3CurrentPosition.
  
          nMp3CurrentPosition += nByteCount;

          // If VS1053_DREQ just transitioned low (e.g. the buffer just became
          // full), give up some time.
          
          if(! digitalRead(VS1053_DREQ))
          {
            // Give up some time.  I measured the time between the falling edge
            // of VS1053_DREQ to the next rising edge of VS1053_DREQ to be about
            // 23.6ms for each VS1053_DATA_XFER_SIZE size data transfer, so any
            // delay "comfortably" less than 23.6ms should work, I chose 20ms.
            
            vTaskDelay(20);
          }
        }
        else
        {
          // No more mp3 data available, stop play.
          
          bMp3PlackEnabled = false;

          // Debug.
        
          Serial_printf("taskMp3Play(): end of mp3 \"%s\".\n", chMp3NameLocal);
        }
     }
  }

  // taskMp3Play no longer running.

  bMp3Playing = false;
  
  // Done.
  
  vTaskDelete(NULL);  
}

//
// taskMp3Start
// Task to start sd card mp3 playbock on the vs1053 via spi.
// Entry   : pointer to name of file to play on sd card
// Returns : nothing
//

void taskMp3Start(void * chMp3Name) 
{
  // Disable mp3 play.

  bMp3PlackEnabled = false;

  // Wait for taskMp3Play to stop.
  
    while(bMp3Playing)
    {
      // taskMp3Play is still running, delay awhile then check again.
      
      vTaskDelay(40);

      // Debug.
      
      Serial_printf("taskMp3Start(): waiting for taskMp3Play() to stop.\n");
    }

  // taskMp3Play has stopped, it is now safe to close the current mp3 file.
  
  fMp3Current.close();
  
  // Reset the vs1053.
  
  vs1053Reset();

  // Open the new mp3.

    // Attempt to open the file.
    
    fMp3Current = SD.open((char *)chMp3Name, "rb");
    if(! fMp3Current)
    {
      // Debug.
      
      Serial_printf("taskMp3Start(): mp3 %s could not be opened.\n", chMp3Name);
      
      // Attempt failed.
  
      vTaskDelete(NULL);  
    }

    // Attempt succeeded, obtain mp3 size and reset mp3 position
    // and task call count.
    
    nMp3CurrentSize = fMp3Current.size();
    nMp3CurrentPosition = 0;
    
  // Wait for vs1053 to become ready.
  
  while(! digitalRead(VS1053_DREQ))
  {
    // Not yet ready, delay.
    
    vTaskDelay(100);
  
    // Check again for ready.
    
    if(! digitalRead(VS1053_DREQ))
    {
      // Still not ready, reset and check again.
      
      vs1053Reset();

      // Debug.
      
      Serial_printf("taskMp3Start(): vs1053 not ready, resetting it again.\n");
    }
  }

  // Enable mp3 play.
    
  bMp3PlackEnabled = true;

  // Start taskMp3Play in cpu core 0.

  xTaskCreatePinnedToCore(taskMp3Play, "taskMp3Play", 4000, chMp3Name, 4, NULL, 0);

  // Debug.
  
  Serial_printf("taskMp3Start(): starting mp3 \"%s\", size %d bytes.\n", chMp3Name, nMp3CurrentSize);

  // End of task.
  
  vTaskDelete(NULL);  
}

//
// taskUpdateClockHands
// Task to perform update of clock hands.
// Entry   : pointer to arguments
// Returns : nothing
//

void  taskUpdateClockHands(void * tmTime)
{
  // Local variables.

  long              nClockwiseMinutes = 0;
  long              nCounterClockwiseMinutes = 0;
  long              nMinutesDelta = 0;
  long              nStepCount = 0;
  long              nStepDirection = 0;
  long              nStepPosition = 0;
  long              nStepPositionN1 = 0;
  long              nTimeInMinutes = 0;
  static long       nTimeInMinutesIndicated = 0;

  // Check for stepper motor availability.

  if(bStepperInUse)
  {
    // Debug.
    
    Serial_printf("taskUpdateClockHands(): stepper motor in use, task terminated.\n");
                   
    // Stepper motor in use, end this task.
    
    vTaskDelete(NULL);  
  }
  
  // Stepper motor not in use, calculate nTimeInMinutes.
  
  nTimeInMinutes = (((((tmClock *)tmTime)->tm_hour + ((tmClock *)tmTime)->tm_isdst) % 12) * 60) + ((tmClock *)tmTime)->tm_min;

  // Check if update is needed.

  if(nTimeInMinutesIndicated != nTimeInMinutes)
  {
    // Clock hands need update, stepper motor is now in use.
    
    bStepperInUse = true;
    
    // Calculate clockwise and counterclockwise time in minutes
    // required to drive indicated time to actual time.
    
    if(nTimeInMinutes > nTimeInMinutesIndicated)
    {
      nClockwiseMinutes = nTimeInMinutes - nTimeInMinutesIndicated;
      nCounterClockwiseMinutes = (nTimeInMinutesIndicated + (12 * 60)) - nTimeInMinutes;
    }
    else if(nTimeInMinutes < nTimeInMinutesIndicated)
    {
      nClockwiseMinutes = (nTimeInMinutes + (12 * 60)) - nTimeInMinutesIndicated;
      nCounterClockwiseMinutes = nTimeInMinutesIndicated - nTimeInMinutes;
    }
    
    // Determine shortest direction.
      
    if(nClockwiseMinutes < nCounterClockwiseMinutes)
    {
      // Clockwise movement is shorter.
      
      nStepDirection = 1;
      nMinutesDelta = nClockwiseMinutes;
    }
    else
    {
      // Counterclockwise movement is shorter.
      
      nStepDirection = -1;
      nMinutesDelta = nCounterClockwiseMinutes;
    }

    // Drive indicated time to time.

    while(nTimeInMinutes != nTimeInMinutesIndicated)
    {
      // Set stepping speed.

      if(nMinutesDelta > 5)
      {
        // > 5 minutes remain, go fast.
        
        nMillisecondsStepHold = MOTOR_STEP_FAST;
      }
      else
      {
        // <= 5 minutes remain, go slow.
        
        nMillisecondsStepHold = MOTOR_STEP_NORMAL;
      }

      // Update minutes to go.
      
      nMinutesDelta --;
      
      // Calculate current position.
      
      nStepPositionN1 = ((nTimeInMinutesIndicated % 60) * MOTOR_STEPS_PER_HOUR) / 60;
      
      // Update minutes.
      
      nTimeInMinutesIndicated = (nTimeInMinutesIndicated + nStepDirection) % (12 * 60);
      nTimeInMinutesIndicated = (nTimeInMinutesIndicated < 0) ? (12UL * 60UL) + nTimeInMinutesIndicated : nTimeInMinutesIndicated;
      
      // Calculate target position.

      nStepPosition = ((nTimeInMinutesIndicated % 60) * MOTOR_STEPS_PER_HOUR) / 60;

      // Calculate step count.
      
      nStepCount = ((nStepDirection > 0) ? nStepPosition - nStepPositionN1 : nStepPositionN1 - nStepPosition);
      nStepCount = (nStepCount < 0) ? MOTOR_STEPS_PER_HOUR + nStepCount : nStepCount;

      // Step the required steps.
      
      while(nStepCount)
      {
        // Step.

        Step(nStepDirection);

        // Count steps.
        
        nStepCount = nStepCount - 1;
      }
    }

    // Remove motor power.
  
    MotorOff();
    
    // Clock hands are updated.
    
    bClockHandsUpdated = true;
                 
    // Finished with the stepper motor.
  
    bStepperInUse = false;
  
    // Debug.
    
    Serial_printf("taskUpdateClockHands(): clock hands updated.\n");
  }
    
  // Finished with task.
  
  vTaskDelete(NULL);  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Arduino.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//
// setup
//

void setup() 
{
#if USE_SERIAL_PRINTF

  // Serial.
  
    Serial.begin(115200);
    while(!Serial){};
  
    Serial_printf("setup(): starting setup.\n");

#endif

  // Pins.
  
    // vs1053 chip select.
    
    pinMode(VS1053_CS, OUTPUT);
    digitalWrite(VS1053_CS, HIGH);
    
    // vs1053 data / control select.
    
    pinMode(VS1053_DCS, OUTPUT);
    digitalWrite(VS1053_DCS, HIGH);
    
    // vs1053 data request.
    
    pinMode(VS1053_DREQ, INPUT);
    
    // Stepper motor contoller.
    
    pinMode(MOTOR_PHASE_A, OUTPUT);
    pinMode(MOTOR_PHASE_B, OUTPUT);
    pinMode(MOTOR_PHASE_C, OUTPUT);
    pinMode(MOTOR_PHASE_D, OUTPUT);
    
    // esp32 on board led.
    
    pinMode(LED_PIN, OUTPUT);
    
    // 12:00 o'clock position reed switch.
    
    pinMode(SWITCH_PIN, INPUT_PULLUP);

#if HOME_SWITCH_CALIBRATE

  // Home switch position calibration.
  
    // Rotate counter clockwise 90 degrees.
    
    nMillisecondsStepHold = MOTOR_STEP_FAST;
    for(int nCount = 0; nCount < MOTOR_STEPS_PER_HOUR / 4; nCount ++)
    {
      Step(-1);
    }
    
    // Home the clock.
    
    ClockHome(& tmTime);
    delay(100);
    while(bStepperInUse)
    {
      delay(100);
    }
    
    // On board led on.
    
    digitalWrite(LED_PIN, HIGH);
    
    // Rotate clockwise until the 12:00 switch activates.
    
    nMillisecondsStepHold = MOTOR_STEP_FAST;
    while(digitalRead(SWITCH_PIN))
    {
      Step(1);
    }
    
    // On board led off.
    
    digitalWrite(LED_PIN, LOW);
    
    // Motor off.
    
    MotorOff();
    
    // Loop forever (reset the esp32 to restart the test).
    
    while(1){};
  
#endif

  // Clock hands.

  ClockHome(& tmTime);

  // Wifi.

    // Debug.
    
    Serial_printf("setup(): initializing wifi connection.\n");
    
    // Start wifi.
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(chSSID, chPassword);

    // Check wifi status.
  
      // Debug.
      
      Serial_printf("setup(): awaiting wifi connection.");
  
      // Wait for wifi to connect.
      
      while(WiFi.status() != WL_CONNECTED)
      {
          Serial_printf(".");
          delay(500);
      }

    // Connected.
    
    // Debug.
    
    Serial_printf("\nsetup(): wifi connected to \"%s\", ip address = %s.\n", chSSID, WiFi.localIP().toString().c_str());
    
  // Sd card service.
  //
  // Notes:
  //
  //  1) The sd card library uses hardware spi, thus it initializes the
  //     spi hardware and as such SPI.begin() is not necessary.

    // Begin sd service.
    
    if(!SD.begin(SD_CS))
    {
      // Debug.
      
      Serial_printf("setup(): sd service failed or sd card not present.\n");

      // sd service failed or sd card not present.
      
      while (1);
    }
    
    // sd service started.

    // Debug.
    
    Serial_printf("setup(): sd card found.\n");
    
  // Ntp time.

    // Begin udp service.
    
    Udp.begin(UDP_PORT);

  // Interrupt service.
  
    // Create onTimer() and loop() semaphore.
    
    timerLoopSemaphore = xSemaphoreCreateBinary();
    
    // Use timer 0 (4 available from 0 through 3), with a prescale of 80.
    
    timerLoop = timerBegin(0, 80, true);
    
    // Attach onTimer() to timer.
    
    timerAttachInterrupt(timerLoop, & onTimer, true);
    
    // Set alarm to call onTimer function every second (value in microseconds)
    // with repeat (third parameter).
    
    timerAlarmWrite(timerLoop, 1000000, true);
    
    // Start the alarm.
    
    timerAlarmEnable(timerLoop);
  
  // Wait for the clock to home.

  while(bStepperInUse)
  {
    delay(4000);
    Serial_printf("setup(): waiting for clock to home.\n");
  }
  Serial_printf("setup(): clock homed.\n");
  
  // End of setup.

  Serial_printf("setup(): setup complete.\n");
}

//
// loop
//
// Notes:
//
//  1) On the first pass through the loop:
//
//      a) setup() has intialized the time to 0, which represents the date and
//         time of Thu, Jan 1st, 1970, 12:00:00am, Dst off.
//
//      b) The call to UpdateNtp() by UpdateTime() immediatedly sends the first
//         ntp request since minutes == 0.  While waiting for the ntp server to
//         reply, the clock will update from the initialized time until ntp time
//         is received from the ntp server, thus the clock will be incorrect.
//
//      c) UpdateChimes() will start playing the top of hour chimes without the
//         hours count chimes, detected by the local variable bChimeColdStart.
//         This is simply the power up tune to indicate the audio is functional.
//

void loop() 
{
  // Wait for alarm tick (set to 1hz. in setup()).
  
  if(xSemaphoreTake(timerLoopSemaphore, portMAX_DELAY) == pdTRUE)
  {
    // For testing, read the current microseconds and set the led pin high (for
    // oscilliscope use).

    unsigned long ulMicrosStart = micros();
    digitalWrite(LED_PIN, HIGH);
    
    // Update time.

    UpdateTime(& tmTime);
    
    // Update chimes.
  
    UpdateChimes(& tmTime);
        
    // Update clock hands.
    
    if(!bStepperInUse)
    {
      // bStepperInUse not in use, use it.
      //
      // Note that during dst transitions, the clock hands will move forward
      // (off to on) or backward (on to off) one hour.  During that foward or
      // backward movement, although the software will not allow it, another
      // taskUpdateClockHands should not be created until the clock completes
      // the hour transition.
      
      xTaskCreatePinnedToCore(taskUpdateClockHands, "taskUpdateClockHands", 4000, & tmTime, 2, NULL, 1);
    }

    // For testing the loop execution time with an oscilliscope,
    // at the end of each loop pass turn the led off.
    
    digitalWrite(LED_PIN, LOW);

    // For testing via the Arduino serial monitor, capture the time.

    unsigned long ulMicrosEnd = micros();

    // For testing, update statistics.

      // Local variables.

      #define AVERAGE_DELTAS 40

      static unsigned long ulDeltas[AVERAGE_DELTAS];
      unsigned long        ulDeltasAverage = 0;
      static unsigned long ulDeltasCount = 0;
      static unsigned long ulDeltasPointer = 0;

      // Statistic calculations start after clock hands are updated.
      
      if(bClockHandsUpdated)
      {
        // Calculate loop time.
  
          // Adjust for overflow.
          
          if(ulMicrosEnd < ulMicrosStart)
          {
            // Micros overflowed.
    
            ulMicrosEnd = 4294967295UL - (ulMicrosStart - ulMicrosEnd) + 1;
            ulMicrosStart = 0UL;
          }
  
          // Calculate loop time.
          
          ulMicrosCurrent = ulMicrosEnd - ulMicrosStart;
  
          // Update high and low loop times.
    
          if(ulMicrosLow > ulMicrosCurrent)
            ulMicrosLow = ulMicrosCurrent;
          if(ulMicrosHigh < ulMicrosCurrent)
            ulMicrosHigh = ulMicrosCurrent;
            
          // Update average loop time.
    
            // Add the current loop time to the average array.
            
            ulDeltas[((ulDeltasPointer += 1) %= AVERAGE_DELTAS)] = ulMicrosCurrent;
    
            // Update the count of deltas in the averages array.
            
            ulDeltasCount = (ulDeltasCount >= AVERAGE_DELTAS) ? AVERAGE_DELTAS : ulDeltasCount + 1;
    
            // Sum the contents of the deltas array.
            
            for(int nCount = 0; nCount < ulDeltasCount; nCount ++)
            {
              ulDeltasAverage += ulDeltas[nCount];
            }
    
            // Calculate the average.
            
            ulDeltasAverage /= ulDeltasCount;
            ulMicrosAverage = ulDeltasAverage;
  
          // Clock stats are now valid.
  
          bTimeStatsValid = true;
      }
        
    // Debug.
    
    Serial_printf("loop(): date and time  : %s, %s %d%s, %4d, %02d:%02d:%02d%s, dst is %s.\nloop(): execution time : cur. = %luus, %d cnt. avg. = %lus, high = %luus, low = %luus.\n\n",
                   chDaysOfWeek[tmTime.tm_wday],
                   chMonth[tmTime.tm_mon],
                   tmTime.tm_mday,
                   (tmTime.tm_mday == 1) ? "st" : (tmTime.tm_mday == 2) ? "nd" : (tmTime.tm_mday == 3) ? "rd" : "th",
                   tmTime.tm_year,
                   ((tmTime.tm_hour + tmTime.tm_isdst) % 12) ? ((tmTime.tm_hour + tmTime.tm_isdst) % 12) : 12,
                   tmTime.tm_min,
                   tmTime.tm_sec,
                   (((tmTime.tm_hour + tmTime.tm_isdst) % 24) >= 12) ? "pm" : "am",
                   (tmTime.tm_isdst) ? "on" : "off",
                   ulMicrosEnd - ulMicrosStart,
                   AVERAGE_DELTAS,
                   ulDeltasAverage,
                   ulMicrosHigh,
                   ulMicrosLow);
  }
  else
  {
    Serial_printf("loop(): this should never execute.\n");
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Utilities.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//
// ClockHome
// Home the clock to 12:00.
// Entry   : pointer to tmClock structure
// Returns : nothing
//

void  ClockHome(tmClock * tmTime)
{
  // Start taskHome with tmTime.
  
  xTaskCreatePinnedToCore(taskClockHome, "taskClockHome", 4000, tmTime, 2, NULL, 1);
}

//
// DateAndTimeToSeconds
// Convert year, month, day, hours, minutes and seconds to seconds since 1970.
// Entry   : pointer to tmClock structure
// Returns : time in seconds since 1970
// Notes   :
//
// 1) Derived from this excellent document:  http://howardhinnant.github.io/date_algorithms.html
//

unsigned long DateAndTimeToSeconds(tmClock * tmTime)
{
    long          lYear = (long)tmTime->tm_year - ((((long)tmTime->tm_mon + 1UL) <= 2UL) ? 1UL : 0UL);
    unsigned long ulEra = ((lYear >= 0UL) ? lYear : lYear - 399UL) / 400UL;
    unsigned long ulMonthOfEra = (lYear - (ulEra * 400UL));
    unsigned long ulDayOfYear = (153UL * (((long)tmTime->tm_mon + 1UL) + (((long)tmTime->tm_mon + 1UL) > 2UL ? -3UL : 9UL)) + 2UL) / 5UL + tmTime->tm_mday - 1UL;
    unsigned long ulDayOfEra = (ulMonthOfEra * 365UL) + (ulMonthOfEra / 4UL) - (ulMonthOfEra / 100UL) + ulDayOfYear;
    
    return (((ulEra * 146097UL + ulDayOfEra - 719468UL) * (SECONDS_PER_DAY)) +
            ((long)tmTime->tm_hour * SECONDS_PER_HOUR) +
            ((long)tmTime->tm_min * SECONDS_PER_MINUTE) +
             (long)tmTime->tm_sec);
}

//
// FirstSunday
// Given the week day of the first day of the month, returns the day of the month of the first Sunday.
// Entry   : weekday (0 through 6 for Sunday through Saturday) of the first day of the month
// Returns : monthday (1 through 7) of the first Sunday of the current month
// Notes   :
//
//  1) Example:
//
//    a) If the first weekday of the month == 0 (Sun) then return 1 (first Sun on the 1st day of the month)
//    b) If the first weekday of the month == 1 (Mon) then return 7 (first Sun on the 7th day of the month)
//    c) If the first weekday of the month == 2 (Tue) then return 6 (first Sun on the 6th day of the month)
//    d) If the first weekday of the month == 3 (Wed) then return 5 (first Sun on the 5th day of the month)
//    e) If the first weekday of the month == 4 (Thu) then return 4 (first Sun on the 4th day of the month)
//    f) If the first weekday of the month == 5 (Fri) then return 3 (first Sun on the 3rd day of the month)
//    g) If the first weekday of the month == 6 (Sat) then return 2 (first Sun on the 2nd day of the month)
//

int   FirstSunday(int nDayOfWeek)
{
  static const int  nFirstSundayDayOfMonthByDayOfWeek[] = {1, 7, 6, 5, 4, 3, 2};
  return(nFirstSundayDayOfMonthByDayOfWeek[nDayOfWeek % 6]);
}

//
// IsDst
// Returns true if daylight savings time is on, false is not.
// Entry   : pointer to tmClock structure
// Returns : 1 if dst is on, 0 if not
// Notes   :
//
// 1) The return value is true if (current time in seconds >= daylight savings time
//    start in seconds) and (current time in seconds < daylight savings time end
//    in seconds), false if not.
//

int  IsDst(tmClock * tmTime)
{
  if((tmTime->ulTime >= tmTime->ulDstStart) && (tmTime->ulTime < tmTime->ulDstEnd) && nDstEndMonth && nDstStartMonth)
  {
    // dst is on.
    
    return 1;
  }
  else
  {
    // dst is off.
    
    return 0;
  }
}

//
// MotorOff
// Turn off motor drive.
// Entry   : nothing
// Returns : nothing
//

void  MotorOff()
{
  digitalWrite(MOTOR_PHASE_A, LOW);
  digitalWrite(MOTOR_PHASE_B, LOW);
  digitalWrite(MOTOR_PHASE_C, LOW);
  digitalWrite(MOTOR_PHASE_D, LOW);
}

//
// Mp3Start
// Start sd card mp3 playbock on the vs1053 via spi.
// Entry   : pointer to name of file to play on sd card
// Returns : nothing
//

void Mp3Start(char * chMp3Name) 
{
  // Set chMp3CurrentName.

  strcpy(chMp3CurrentName, chMp3Name);
  
  // Start taskMp3Start with chMp3CurrentName.
  
  xTaskCreatePinnedToCore(taskMp3Start, "taskMp3Start", 4000, & chMp3CurrentName, 0, NULL, 0);
}

//
// SecondSunday
// Given the week day of the first day of the month, returns the day of the month of the second Sunday.
// Entry   : weekday (0 through 6 for Sunday through Saturday) of the first day of the current month
// Returns : monthday (8  through 14) of the first Sunday of the current month
// Notes   :
//
//  1) Example:
//
//    a) If the first weekday of the month == 0 (Sun) then return  8 (second Sun on the 8th day of the month)
//    b) If the first weekday of the month == 1 (Mon) then return 14 (second Sun on the 14th day of the month)
//    c) If the first weekday of the month == 2 (Tue) then return 13 (second Sun on the 13th day of the month)
//    d) If the first weekday of the month == 3 (Wed) then return 12 (second Sun on the 12th day of the month)
//    e) If the first weekday of the month == 4 (Thu) then return 11 (second Sun on the 11th day of the month)
//    f) If the first weekday of the month == 5 (Fri) then return 10 (second Sun on the 10th day of the month)
//    g) If the first weekday of the month == 6 (Sat) then return  9 (second Sun on the 9th day of the month)
//

int   SecondSunday(int nDayOfWeek)
{
  return (FirstSunday(nDayOfWeek) + 7);
}

//
// SecondsToDateAndTime
// Convert seconds since 1970 to year, month, day, hours, minutes and seconds.
// Entry   : pointer to tmClock structure
// Returns : nothing
// Notes   :
//
// 1) Derived from this excellent document:  http://howardhinnant.github.io/date_algorithms.html
//

void  SecondsToDateAndTime(tmClock * tmTime)
{
    unsigned long ulZ = (tmTime->ulTime / SECONDS_PER_DAY) + 719468UL;
    unsigned long ulEra = ((ulZ >= 0UL) ? ulZ : ulZ - 146096UL) / 146097UL;
    unsigned long ulDayOfEra = (ulZ - ulEra * 146097UL);
    unsigned long ulYearOfEra = (ulDayOfEra - (ulDayOfEra / 1460UL) + (ulDayOfEra / 36524UL) - (ulDayOfEra / 146096UL)) / 365UL;
    unsigned long ulYear = ulYearOfEra + (ulEra * 400UL);
    unsigned long ulDayOfYear = ulDayOfEra - ((365UL * ulYearOfEra) + (ulYearOfEra / 4UL) - (ulYearOfEra / 100UL));
    unsigned long ulMonthNumber = (5UL * ulDayOfYear + 2UL) / 153UL;
    unsigned long ulDayOfMonth = ulDayOfYear - (153UL * ulMonthNumber + 2UL) / 5UL + 1UL;
    unsigned long ulMonthOfYear = ulMonthNumber + (ulMonthNumber < 10UL ? 3UL : -9UL);
    long          lDays = (tmTime->ulTime / SECONDS_PER_DAY);

    tmTime->tm_year = ulYear + ((ulMonthOfYear <= 2) ? 1 : 0);
    tmTime->tm_mon = ulMonthOfYear - 1;
    tmTime->tm_mday = ulDayOfMonth;
    tmTime->tm_hour = (tmTime->ulTime % SECONDS_PER_DAY) / SECONDS_PER_HOUR;
    tmTime->tm_min = (tmTime->ulTime % SECONDS_PER_HOUR) / SECONDS_PER_MINUTE;
    tmTime->tm_sec = (tmTime->ulTime % SECONDS_PER_MINUTE);
    tmTime->tm_wday = (lDays >= - 4) ? (lDays + 4) % 7 : (lDays + 5) % 7 + 6;
    
    return; 
}

//
// Step
// Step the stepper motor.
// Entry   : direction (1 = clockwise, -1 = counter clockwise)
// Returns : nothing
//
// Notes   : 1) for this stepper motor, 1 step is 1 / MOTOR_STEPS_PER_HOUR degrees.
//
//           2) forward clock motion is performed in 8 steps:
//
//              a) Phase d
//              b) Phase d and c
//              c) phase c
//              d) phase c and b
//              e) phase b
//              f) phase b and a
//              g) phase a
//              h) phase a and d
//
//            3) Reverse clock motion is performed in the reverse order of 2).
//

void  Step(int nDirection)
{
  // Local variables.
  
  static  int nPhase = 0;

  // Update phase.
  
  nPhase = ((nDirection < 0) ? (nPhase - 1) : (nPhase + 1)) & 7;

  // Step this phase.
  
  switch(nPhase)
  {
    case 0:
    {
      digitalWrite(MOTOR_PHASE_D, HIGH);
      digitalWrite(MOTOR_PHASE_C, LOW);
      digitalWrite(MOTOR_PHASE_B, LOW);
      digitalWrite(MOTOR_PHASE_A, LOW);
    }
    break;

    case 1:
    {
      digitalWrite(MOTOR_PHASE_D, HIGH);
      digitalWrite(MOTOR_PHASE_C, HIGH);
      digitalWrite(MOTOR_PHASE_B, LOW);
      digitalWrite(MOTOR_PHASE_A, LOW);
    }
    break;

    case 2:
    {
      digitalWrite(MOTOR_PHASE_D, LOW);
      digitalWrite(MOTOR_PHASE_C, HIGH);
      digitalWrite(MOTOR_PHASE_B, LOW);
      digitalWrite(MOTOR_PHASE_A, LOW);
    }
    break;

    case 3:
    {
      digitalWrite(MOTOR_PHASE_D, LOW);
      digitalWrite(MOTOR_PHASE_C, HIGH);
      digitalWrite(MOTOR_PHASE_B, HIGH);
      digitalWrite(MOTOR_PHASE_A, LOW);
    }
    break;

    case 4:
    {
      digitalWrite(MOTOR_PHASE_D, LOW);
      digitalWrite(MOTOR_PHASE_C, LOW);
      digitalWrite(MOTOR_PHASE_B, HIGH);
      digitalWrite(MOTOR_PHASE_A, LOW);
    }
    break;

    case 5:
    {
      digitalWrite(MOTOR_PHASE_D, LOW);
      digitalWrite(MOTOR_PHASE_C, LOW);
      digitalWrite(MOTOR_PHASE_B, HIGH);
      digitalWrite(MOTOR_PHASE_A, HIGH);
    }
    break;

    case 6:
    {
      digitalWrite(MOTOR_PHASE_D, LOW);
      digitalWrite(MOTOR_PHASE_C, LOW);
      digitalWrite(MOTOR_PHASE_B, LOW);
      digitalWrite(MOTOR_PHASE_A, HIGH);
    }
    break;

    case 7:
    {
      digitalWrite(MOTOR_PHASE_D, HIGH);
      digitalWrite(MOTOR_PHASE_C, LOW);
      digitalWrite(MOTOR_PHASE_B, LOW);
      digitalWrite(MOTOR_PHASE_A, HIGH);
    }
    break;
  }

  // Hold this step for nMillisecondsStepHold milliseconds.
   
  delay(nMillisecondsStepHold);
}

//
// UpdateChimes
// Update chimes.
// Entry   : pointer to tmClock structure
// Returns : nothing
//

void  UpdateChimes(tmClock * tmTime)
{
  // Local variables.
  
  static bool           bChimeColdStart = true; // cold start wakeup chime (initialized to true to avoid hour chime count on cold start)
  static int            nChimeHours = 0;        // chime hours count (0 if off, 1 through 12 for hour chime count)
  static int            nMinutesN1 = -1;        // minutes n - 1 (initialized to -1 in order to immediately detect a change in minutes)
  static bool           bMp3PlayingN1 = false;  // bMp3Playing n - 1
  
  // Start chimes only if there on no hours chimes being
  // played, and minutes have changed.
  
  if((nChimeHours == 0) && (nMinutesN1 != tmTime->tm_min))
  {
    // No hours chime is in process and minutes changed,
    // update minutes n - 1.

    nMinutesN1 = tmTime->tm_min;
    
    // Check for chime start on the quarter hour.

    switch(tmTime->tm_min)
    {
      case 0:
      {
        // Minutes == 0, start hour chime.
        
        Mp3Start("/12.mp3");
    
        // Then after hour mp3 plays, chime the hour count if not cold start chime.

        if(bChimeColdStart)
        {
          // Cold start chime, just play the top of hour.
          
          bChimeColdStart = false;
        }
        else
        {
          // Not cold start chime, chime the hour count.

          nChimeHours = (((tmTime->tm_hour + tmTime->tm_isdst) % 12) == 0) ? 12 : (tmTime->tm_hour + tmTime->tm_isdst) % 12;
        }
      }
      break;
      
      case 15:
      {
        // Minutes == 15, start quarter hour chime.
        
        Mp3Start("/15.mp3");
      }
      break;
    
      case 30:
      {
        // Minutes == 30, start half hour chime.
        
        Mp3Start("/30.mp3");
      }
      break;
    
      case 45:
      {
        // Minutes == 45, start three quarter hour chime.
        
        Mp3Start("/45.mp3");
      }
      break;
    }
  }

  // Check nChimeHours.

  if(nChimeHours > 0)
  {
    // nChimeHours > 0, check for end of top of hour mp3 by waiting for
    // bMp3Playing to transistion from on to off.
    //
    // Note since StartMp3() only starts taskStartMp3(), the bMp3Playing
    // flag will not be set until the os actually starts taskStartMp3()
    // thus here we check for a transition of bMp3Playing from on to off,
    // indicating that mp3 playback has indeed ended.
    
    if(bMp3PlayingN1 != bMp3Playing)
    {
      // Transition, update n-1.
      
      bMp3PlayingN1 = bMp3Playing;
  
      // Check transition.
      
      if(!bMp3Playing)
      {
        // On to off transition thus top of hour mp3 has ended.
        //
        // Note when nChimeHours is > 0 it contains a valid "chime count" from 1
        // through 12.  There are 12 hour count chime files, "/Hours1.mp3" through
        // "/Hours12.mp3", on the sd card.  Build the mp3 filename for the file
        // associated with the hour chime count contained in nChimeHours.
        
        char  chBuffer[81];
        sprintf(chBuffer, "/Hours%d.mp3", nChimeHours);
  
        // Debug.
        
        Serial_printf("UpdateChimes(): hours chime file is \"%s\".\n", chBuffer);
        
        // Start the mp3.
        
        Mp3Start(chBuffer);
  
        // End of chime hours
        
        nChimeHours = 0;
      }
      else
      {
        // Off to on transition.
        
      }
    }
    else
    {
      // No transition.
      
    }
  }
  else
  {
    // No nChimeHours.
    
  }
}

//
// UpdateDstStartAndEnd
// Update the daylight savings time variables ulDstStart and ulDstEnd.
// Entry   : the year in which to generate dst variables
//         : pointer to tmClock structure
// Returns : nothing
//

void UpdateDstStartAndEnd(int nYear, tmClock * tmTime)
{
  // Local variables.
  
  tmClock  tmTimeLocal;

  // Both user settings months must be non-zero for dst to be processed.

  if(nDstStartMonth && nDstEndMonth)
  {
    // User desires dst, calculate the start date of dst in seconds for this year.
  
      // Obtain the start seconds.
  
        // Obtain the seconds since 1970 for the start month of the year
        // (see tmClock structure for element details).
        
        tmTimeLocal = {0, 0, 0, 1, (nDstStartMonth - 1) % 12, nYear, 0, 0, 0, 0, 0, false};                        
        tmTimeLocal.ulTime = DateAndTimeToSeconds(& tmTimeLocal);
  
        // Fill in the remaining tmTime elements.
        
        SecondsToDateAndTime(& tmTimeLocal);
  
        // Set tm_mday and tm_hour based on user input.
        //
        // Note if the user enters 0 for the day, then use SecondSunday(),
        // else use the user day.
        
        tmTimeLocal.tm_mday = (nDstStartDay) ? nDstStartDay : SecondSunday(tmTimeLocal.tm_wday);
        tmTimeLocal.tm_hour = nDstStartHour % 24;
  
        // Set tmTime->ulDstStart.
        
        tmTime->ulDstStart = DateAndTimeToSeconds(& tmTimeLocal);
                     
      // Obtain the end seconds.
  
        // Obtain the seconds since 1970 for the end month of the year
        // (see tmClock structure for element details).
        
        tmTimeLocal = {0, 0, 0, 1, (nDstEndMonth - 1) % 12, nYear, 0, 0, 0, 0, 0, false};                        
        tmTimeLocal.ulTime = DateAndTimeToSeconds(& tmTimeLocal);
      
        // Fill in the remaining tmTime elements.
        
        SecondsToDateAndTime(& tmTimeLocal);
  
        // Set tm_mday and tm_hour based on user input.
        //
        // Note if the user enters 0 for the day, then use FirstSunday(),
        // else use the user day.  Note also the clock logic runs on
        // ntp time which does not account for dst, so the end hour is
        // set to the user end hour - 1.
  
        tmTimeLocal.tm_mday = (nDstEndDay) ? nDstEndDay : FirstSunday(tmTimeLocal.tm_wday);
        tmTimeLocal.tm_hour = (nDstEndHour - 1) % 24;
        
        // Set tmTime->ulDstEnd.
        
        tmTime->ulDstEnd = DateAndTimeToSeconds(& tmTimeLocal);
  
    // Dst start and end are now valid.
  
    tmTime->bDstValid = true;
  }
  else
  {
    // One or both user settings months are zero, no dst processing.
    
  }
}

//
// UpdateNtp
// Update ntp time.
// Entry   : pointer to tmClock structure
// Returns : true if time updated, false if not
// Notes   :
//
//  1) UpdateNtp operates in one of three modes, REQUEST, SENT and RECEIVED:
//
//    a) If nNtpPacketMode == REQUEST, UpdateNtp will create and send an ntp
//       packet to request the time, then set nNtpPacketMode = SENT to indicate
//       the packet has been sent.
//
//    b) If nNtpPacketMode == SENT, UpdateNtp will await a response from the
//       ntp time server.  If a response is received, UpdateNtp will convert
//       the received time from ntp time to unix time and update the tmClock
//       structure, then set nNtpPacketMode = RECEIVED to indicate that the
//       time has been received from the ntp time server.  If a response is
//       not received after five seconds, UpdateNtp will set nNtpPacketMode =
//       REQUEST to request another packet.
//
//    c) If nNtpPacketMode == RECEIVED, UpdateNtp will wait for the top of the
//       next hour, then set nNtpPacketMode = REQUEST.
//
//  2) If everything works as planned:
//
//    a) On cold start, nNtpPacketMode = REQUEST, and bNtpTimeRequestEnabled = false.
//
//    b) UpdateNtp case REQUEST sends a time request to the ntp server, then sets
//       nNtpPacketMode = SENT.
//
//    c) UpdateNtp case SENT waits for the ntp time server response.
//
//    d) If UpdateNtp case SENT receives ntp time from the server, it converts the
//       received time from ntp time to unix time, updates the tmClock structure,
//       then sets nNtpPacketMode = RECEIVED.
//
//    e) If UpdateNtp case SENT does not receive ntp time from the server, it downcounts
//       nNtpPacketDelay until zero, at which time it sets nNtpPacketMode = REQUEST to
//       return to 2.b and start the process again.
//
//    f) UpdateNtp case RECEIVED waits for the top of the next hour, then sets
//       nNtpPacketMode = REQUEST to start the process again.
//
//    g) If all goes well (e.g. the ntp time server responds), steps 2.b, 2.c, 2.d and
//       2.f are continuously repeated at a rate of once per hour.
//
//    h) If all does not go well (e.g. the ntp time server does not respond), steps 2.b,
//       2.c, and 2.e are continuously repeated at a rate of 5hz until the ntp time
//       server responds.
//
//

bool  UpdateNtp(tmClock * tmTime)
{
  // Local variables.

  static int              nNtpPacketDelay        = PACKET_DELAY_RESET;
  static NtpPacketMode    nNtpPacketMode         = REQUEST;             // initialized to REQUEST to immediately request ntp time on cold start

  // Determine the current mode.
  
  switch(nNtpPacketMode)
  {
    case REQUEST:
    {
      // REQUEST mode, create and initialize ntp packet.
    
      byte chNtpPacket[NTP_PACKET_LENGTH];
      memset(chNtpPacket, 0, NTP_PACKET_LENGTH);
      chNtpPacket[0]  = 0b00011011;
  
      // Send the ntp packet (see https://tf.nist.gov/tf-cgi/servers.cgi for other ip addresses).
      
      IPAddress ipNtpServer(129, 6, 15, 29);
      Udp.beginPacket(ipNtpServer, 123);
      Udp.write(chNtpPacket, NTP_PACKET_LENGTH);
      Udp.endPacket();
  
      // Packet has been sent, on the next pass...

        // UpdateNtp is in SENT mode,
        
        nNtpPacketMode = SENT;

        // and initialize nNtpPacketDelay for possible UpdateNtp SENT mode
        // ntp timeouts.
        
        nNtpPacketDelay = PACKET_DELAY_RESET;

      // Debug.
      
      Serial_printf("UpdateNtp(): ntp packet sent.\n");
    }
    break;

    case SENT:
    {
      // SENT mode, an ntp time request has been sent, wait for a response
      // from the ntp time server.
      
      if(Udp.parsePacket())
      {
        // Server responded, read the packet.
    
        Udp.read(chNtpPacket, NTP_PACKET_LENGTH);

        // Obtain the time from the packet, convert to Unix time, and adjust for the time zone.
        //
        // Unix time is indicated as the number of seconds since 1/1/1970 at 00:00.
        // NTP time is indicated as the number of seconds since 1/1/1900 at 00:00.
        // Hence there is a 70 year difference between the two, and 17 leap years
        // during those 70 years.
        //
        // The following equation obtains the ntp seconds bytes then converts ntp
        // seconds to unix seconds by subtracting from the ntp time in seconds the
        // value (((70 * 356) + 17) * SECONDS_PER_DAY) where:
        //
        //  1) 70 is the number of years between unix and ntp time.
        //  2) 365 is the number of days in a year.
        //  3) 17 is the number of leap years during the 70 year difference.
  
        unsigned long ulCurrentSeconds = tmTime->ulTime;
        
        tmTime->ulTime = ((unsigned long)chNtpPacket[40] << 24) +       // bits 24 through 31 of ntp time
                         ((unsigned long)chNtpPacket[41] << 16) +       // bits 16 through 23 of ntp time
                         ((unsigned long)chNtpPacket[42] <<  8) +       // bits  8 through 15 of ntp time
                         ((unsigned long)chNtpPacket[43]) -             // bits  0 through  7 of ntp time
                         (((70UL * 365UL) + 17UL) * SECONDS_PER_DAY) +  // ntp to unix conversion
                         (nTimeZone * SECONDS_PER_HOUR) +               // time zone adjustment
                         2UL;                                           // compenstation for ntp request delay
                         
        // Received seconds, update date and time.
  
        SecondsToDateAndTime(tmTime);
  
        // On the next pass, UpdateNtp is in RECEIVED mode.
    
        nNtpPacketMode = RECEIVED;
      
        // Debug.
        
        Serial_printf("UpdateNtp(): ntp packet received, time in seconds = %lu, delta = %lu.\n",
                      tmTime->ulTime,
                     (tmTime->ulTime >= ulCurrentSeconds) ? (tmTime->ulTime - ulCurrentSeconds) : (ulCurrentSeconds - tmTime->ulTime));
        
        // Time updated.
  
        return true;
      }
      else
      {
        // Server has not yet responded, down count nNtpPacketDelay.
        
        nNtpPacketDelay = nNtpPacketDelay - 1;
        
        // Check nNtpPacketDelay.
        
        if(nNtpPacketDelay <= 0)
        {
          // Timeout, on the next pass, place UpdateNtp in REQUEST mode
          // to try again.
          
          nNtpPacketMode = REQUEST;
          
          // Debug.
          
          Serial_printf("UpdateNtp(): ntp time packet timeout.\n");
        }
      }
    }
    break;
    
    case RECEIVED:
    {
      // RECEIVED mode, ntp time request has been sent and received.  Wait
      // until the top of the next hour to return to REQUEST mode.

      // Local variables.
      
      static bool bNtpTimeRequestEnabled = false;   // latch initialized to false for cold start
      
      // Check minutes.
      
      if(tmTime->tm_min == 0)
      {
        // tmTime->tm_min == 0 (top of hour) will occur for 60 seconds.  To
        // avoid sending 60 ntp time request packets during this time, check
        // if ntp time has already been requested for this hour.
        //
        // I know, I know, you're asking why not check for tmTime->tm_sec ==
        // 0?  In the unusual but plausible event where the clock falls 1 or
        // more seconds behind per hour, the check for tmTime->tm_sec == 0
        // would fail so this code checks for tmTime->tm_min == 0 along with
        // the latching variable bNtpTimeRequestEnabled used as a one shot
        // per hour latch.
        
        if(bNtpTimeRequestEnabled)
        {
          // Entering a new hour, place UpdateNtp in REQUEST mode for the next
          // pass.
          
          nNtpPacketMode = REQUEST;

          // Then dissable further nNtpPacketMode = REQUEST for the remainder
          // of tmTime->tm_min == 0 for the new hour.
          
          bNtpTimeRequestEnabled = false;

          // Debug.
          
          Serial_printf("UpdateNtp(): top of hour, triggering ntp time request.\n");
        }
        else
        {
          // Still in tmTime->tm_min == 0 for the new hour, do nothing, just a comment.

        }
      }
      else
      {
        // No longer in tmTime->tm_min == 0 for the new hour, enable ntp time request
        // for the next hour.
        
        bNtpTimeRequestEnabled = true;

      }
    }
    break;
    
  }
  
  // Time not updated.

  return false;
}
 
//
// UpdateTime
// Update time.
// Entry   : pointer to tmClock structure
// Returns : nothing
//

void  UpdateTime(tmClock * tmTime)
{
  // Check for an ntp update.
  
  if(UpdateNtp(tmTime))
  {
    // UpdateNtp() did update time, update daylight savings start and end times.
    //
    // Note UpdateNtp() updates ntp at the top of each hour, so after each update
    // is a good time for updating the dst start and end times.
    
    UpdateDstStartAndEnd(tmTime->tm_year, tmTime);
  }
  else
  {
    // UpdateNtp() did not update time, manually update time
    // in seconds.
    
    tmTime->ulTime += 1;;
    
    // Update year, month, day, hours, minutes, seconds from ulTime.
    
    SecondsToDateAndTime(tmTime);
  }
    
  // Update daylight savings time status.

  if(tmTime->bDstValid);
    tmTime->tm_isdst = IsDst(tmTime);
}

//
// vs1053Read
// Read data from the vs1035.
// Entry   : address of data in the vs1035 to read
// Returns : data at address
//

unsigned int vs1053Read(unsigned char chAddress) 
{
  // Local variables.
  
  unsigned int nData = 0;

  // Start spi transaction.
  
  SPI.beginTransaction(SPISettings(250000,  MSBFIRST, SPI_MODE0));

  // Select the vs1053.
  
  digitalWrite(VS1053_CS, LOW);  

  // Build the read command.

  unsigned char  chBuffer[2];
  chBuffer[0] = VS1053_SCI_READ;
  chBuffer[1] = chAddress;

  // Send the read command.
  
  SPI.writeBytes(chBuffer, sizeof(chBuffer));

  // Give the vs1053 time to respond.
  
  delayMicroseconds(10);

  // Read the two byte (16 bit) response.
  
    // Read the upper byte.
    
    nData = ((unsigned int)SPI.transfer(0x00) << 8);
   
    // Read the lower byte.
    
    nData |= (unsigned int)SPI.transfer(0x00);
  
  // Deselect the vs1053.
  
  digitalWrite(VS1053_CS, HIGH);

  // End of spi transaction.
  
  SPI.endTransaction();

  // Return the data read.
  
  return nData;
}

//
// vs1053Reset
// Reset the vs1035.
// Entry   : nothing
// Returns : nothing
//

void  vs1053Reset(void)
{
  // Keep trying to reset the vs1053 until success.
  
  while(true)
  {
    // Reset the vs1053.
    
    vs1053Write(VS1053_REG_MODE, VS1053_MODE_SM_SDINEW | VS1053_MODE_SM_RESET);
    delay(100);
    vs1053Write(VS1053_REG_CLOCKF, 0x6000);
    delay(100);
  
    // Validate that vs1053 is reset.
    
    int vs1053Status = (vs1053Read(VS1053_REG_STATUS) >> 4) & 0x0F;
    
    if(vs1053Status == 4)
    {
      // Reset, success break out of loop.
      
      break;
    }
    else
    {
      // Reset failed, delay then try again.
      
      delay(500);

      // Debug.
      
      Serial_printf("vs1053Reset(): vs1053 reset failed, status = %d.\n", vs1053Status);
    }
  }
  
  // Set the vs1053 playback volume.

  vs1053Write(VS1053_REG_VOLUME, (nMp3CurrentVolume << 8) + (nMp3CurrentVolume & 0xff));
}

//
// vs1053Write
// Write data to vs1053
// Entry   : address in vs1053 to write to
//         : data to write
// Returns : nothing
//

void vs1053Write(unsigned char chAddress, unsigned int nData) 
{
  // Begin spi transaction.
  
  SPI.beginTransaction(SPISettings(250000,  MSBFIRST, SPI_MODE0));

  // Select the vs1503.

  digitalWrite(VS1053_CS, LOW);  

  // Build the write command.

  unsigned char  chBuffer[4];
  chBuffer[0] = VS1053_SCI_WRITE;
  chBuffer[1] = chAddress;
  chBuffer[2] = nData >> 8;
  chBuffer[3] = nData & 0xFF;
 
  // Send the write command.

  SPI.writeBytes(chBuffer, sizeof(chBuffer));

  // Deselect the vs1503.
  
  digitalWrite(VS1053_CS, HIGH);

  // End of spi transaction.
  
  SPI.endTransaction();
}


