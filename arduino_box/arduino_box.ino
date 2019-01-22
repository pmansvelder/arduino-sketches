/*
  LiquidCrystal Library - Hello World

  Demonstrates the use a 16x2 LCD display.  The LiquidCrystal
  library works with all LCD displays that are compatible with the
  Hitachi HD44780 driver. There are many of them out there, and you
  can usually tell them by the 16-pin interface.

  The circuit:
   LCD RS pin to digital pin 12
   LCD Enable pin to digital pin 11
   LCD D4 pin to digital pin 5
   LCD D5 pin to digital pin 4
   LCD D6 pin to digital pin 3
   LCD D7 pin to digital pin 2
   LCD R/W pin to ground
   LCD VSS pin to ground
   LCD VCC pin to 5V
   10K resistor:
   ends to +5V and ground
   wiper to LCD VO pin (pin 3)

  Library originally added 18 Apr 2008
  by David A. Mellis
  library modified 5 Jul 2009
  by Limor Fried (http://www.ladyada.net)
  example added 9 Jul 2009
  by Tom Igoe
  modified 22 Nov 2010
  by Tom Igoe
  modified 7 Nov 2016
  by Arturo Guadalupi

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/LiquidCrystalHelloWorld

*/

#define DEBUG 1 // Zet debug mode aan

// include the library code:
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 6

#define outputA 8
#define outputB 9
#define pushButton 10
#define relay 7

#define MQ_present 1 // MQ-x gas sensor
#define MQ_PIN A3 // Vul hier de pin in van de MQ sensor

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

const int temp_max = 300;
const int deBounceTime = 2500;
int deBounce;

long tijd, vorige_tijd;

int counter = 0, temperature = 0, cosensor = 0;
int aState, aLastState;

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

#if defined(DEBUG)
bool debug = true;
#else
bool debug = false;
#endif

void ShowDebug(String tekst) {
  if (debug) {
    Serial.println(tekst);
  }
}

void setup() {
  if (debug) {
    Serial.begin(9600);
    ShowDebug(F("Arduino Box Test"));
  }
  pinMode(outputA, INPUT_PULLUP);
  pinMode(outputB, INPUT_PULLUP);
  pinMode(pushButton, INPUT_PULLUP);
  pinMode(relay, OUTPUT);
  digitalWrite(relay, LOW);
  aLastState = digitalRead(outputB);
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.setCursor(0, 0);
  clearLine();
  lcd.setCursor(0, 0);
  lcd.print("Arduino Box Demo");
  lcd.setCursor(0, 1);
  lcd.print("'arduino_box'");
  vorige_tijd = millis();
#if defined(MQ_present)
  lcd.setCursor(0, 1);
  lcd.print("Set up MQ sensor");
  ShowDebug("Setting up pin " + String(MQ_PIN) + " as MQ gas sensor");
  analogWrite(MQ_PIN, HIGH);
  ShowDebug("Heating up for 60 seconds...");
  delay(60000);
  // now reducing the heating power: turn the heater to approx 1,4V
  ShowDebug("Reducing voltage to 1,4V and heat for 90 seconds...");
  analogWrite(MQ_PIN, 71.4);// 255x1400/5000
  delay(90000);
  ShowDebug("MQ-7 sensor setup done");
#endif
  printState();
  printTemp();
  pinMode(10, INPUT_PULLUP);
  vorige_tijd = 0;
}

void clearLine() {
  for (int i = 1; i < 20 ; i++) {
    lcd.print(" ");
  }
}

void printState() {
  lcd.setCursor(0, 0);
  clearLine();
  lcd.setCursor(0, 0);
  lcd.print("Relais: ");
  if (digitalRead(relay)) {
    lcd.print("On");
  }
  else {
    lcd.print("Off");
  }
}

void printTemp() {
  lcd.setCursor(0, 1);
  clearLine();
  lcd.setCursor(0, 1);
  lcd.print("T: ");
  //  lcd.print(int(temperature));
  ShowDebug(String(sensors.getTempCByIndex(0)));
  lcd.print(sensors.getTempCByIndex(0));
  lcd.print("C");
  lcd.print(" CO: ");
  lcd.print(int(cosensor));
}

void loop() {

  if (digitalRead(pushButton) == LOW) {
    if (deBounce == 0) {
      digitalWrite(relay, !digitalRead(relay));
      deBounce = deBounceTime;
      printState();
    }
    else {
      deBounce --;
    }
  }
  sensors.requestTemperatures();
  aState = digitalRead(outputB);
  cosensor = analogRead(MQ_PIN);

  if (aState != aLastState) {
    if (digitalRead(outputA) != aState) {
      if (temperature < temp_max) {
        temperature ++;
      }
    }
    else {
      if (temperature > 0) {
        temperature --;
      }
    }
    printTemp();
  }
  aLastState = aState;

  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  //  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  tijd = millis() - vorige_tijd;
  if (tijd > 1000) {
    printTemp();
    vorige_tijd = millis();
  }
  //
  //lcd.print(tijd / 1000);
  //if (digitalRead(10) == LOW) {

  //  for (int i = 1; i < 20 ; i++) {
  //    lcd.print(" ");
  //  }
}
