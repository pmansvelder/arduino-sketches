/* Arduino 'slimme meter' P1-port reader.

  This sketch reads data from a Dutch smart meter that is equipped with a P1-port.
  Connect 'RTS' from meter to Arduino pin 5
  Connect 'GND' from meter to Arduino GND
  Connect 'RxD' from meter to Arduino pin 0 (RX) . ==> pin 19

  Pinout: plug with cable down

  + 123456 +
  | |||||| |
  | |||||| |
  |        |

  1 = +5V
  2 = Request (input)
  3 = Data GND
  4 = N.C.
  5 = Data (output)
  6 = GND

  Baudrate 115200, 8N1.
  BS170 transistor & 10k resistor is needed to make data readable if meter spits out inverted data

  A .php file is requested (with consumption numbers in the GET request) every minute (interval set at line #52)
  created by 'ThinkPad' @ Tweakers.net, september 2014

  http://gathering.tweakers.net/forum/list_messages/1601301
*/

//Serial1 on pins 19 (RX) and 18 (TX)

const int requestPin =  5;
char input; // incoming serial data (byte)
bool readnextLine = false;
#define BUFSIZE 75
char buffer[BUFSIZE]; //Buffer for serial data to find \n .
int bufpos = 0;
long mEVLT = 0; //Meter reading Electrics - consumption low tariff
long mEVHT = 0; //Meter reading Electrics - consumption high tariff
long mEAV = 0;  //Meter reading Electrics - Actual consumption
long mG = 0;   //Meter reading Gas
int Tariff = 1; // Current tariff
long Voltage = 0; // Voltage

long lastTime = 0;        // will store last time
long interval = 5000;           // interval at which to read data

void setup() {
  Serial1.begin(115200);
  Serial.begin(115200);
  Serial.println("Arduino telegram decoder ready.");
  delay(1000);
  //Set RTS pin high, so smart meter will start sending telegrams
  pinMode(requestPin, OUTPUT);
  digitalWrite(requestPin, HIGH);
}

void loop() {

  decodeTelegram();

  if (millis() - lastTime > interval) {
    lastTime = millis();
    //send data to PHP/MySQL
    httpRequest();
    //Reset variables to zero for next run
    Voltage = 0;
    Tariff = 0;
    mEVLT = 0;
    mEVHT = 0;
    mEAV = 0;
    mG = 0;

  }
} //Einde loop

void decodeTelegram() {
  long tl = 0;
  long tld = 0;

  if (Serial1.available()) {
    input = Serial1.read();
    char inChar = (char)input;
    // Fill buffer up to and including a new line (\n)
    buffer[bufpos] = input & 127;
    bufpos++;

    if (input == '\n') { // We received a new line (data up to \n)
//      Serial.print(buffer);

      // 0-0:96.14 = Elektra tarief (laag = 1)
      if (sscanf(buffer, "0-0:96.14.0(%d" , &tl) == 1) {
        Tariff = tl;
      }

      // 1-0:32.7 = Netspanning
      if (sscanf(buffer, "1-0:32.7.0(%ld.%ld" , &tl, &tld) == 2) {
        tl += tld;
        Voltage = tl;
      }

      // 1-0:1.8.1 = Elektra verbruik laag tarief (DSMR v4.0)
      if (sscanf(buffer, "1-0:1.8.1(%ld.%ld" , &tl, &tld) == 2) {
        tl *= 1000;
        tl += tld;
        mEVLT = tl;
      }

      // 1-0:1.8.2 = Elektra verbruik hoog tarief (DSMR v4.0)
      if (sscanf(buffer, "1-0:1.8.2(%ld.%ld" , &tl, &tld) == 2) {
        tl *= 1000;
        tl += tld;
        mEVHT = tl;
      }

      // 1-0:1.7.0 = Electricity consumption actual usage (DSMR v4.0)
      if (sscanf(buffer, "1-0:1.7.0(%ld.%ld" , &tl , &tld) == 2)
      {
        mEAV = (tl * 1000) + tld;
      }

      // 0-1:24.2.1 = Gas (DSMR v4.0) on Kaifa MA105 meter
      if (strncmp(buffer, "0-1:24.2.1", strlen("0-1:24.2.1")) == 0) {
        if (sscanf(strrchr(buffer, '(') + 1, "%d.%d", &tl, &tld) == 2) {
          mG = (tl * 1000) + tld;
        }
      }

      // Empty buffer again (whole array)
      for (int i = 0; i < 75; i++)
      {
        buffer[i] = 0;
      }
      bufpos = 0;
    }
  } //Einde 'if AltSerial.available'
} //Einde 'decodeTelegram()' functie

void httpRequest() {
  // if there's a successful connection:

  Serial.print("Tariff=");
  Serial.println(Tariff);
  Serial.print("Voltage=");
  Serial.println(Voltage);
  Serial.print("mEVLT=");
  Serial.println(mEVLT);
  Serial.print("mEVHT=");
  Serial.println(mEVHT);
  Serial.print("mEAV=");
  Serial.println(mEAV);
  Serial.print("mG=");
  Serial.println(mG);

  //Request complete; empty receive buffer

}
