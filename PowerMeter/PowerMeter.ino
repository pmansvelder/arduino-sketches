// Arduino Energy Meter V2.0
// This code is for This code is for Wemos(ESP8266) based Energy monitoring Device
// This code is a modified version of sample code from https://github.com/pieman64/ESPecoMon
// Last updated on 30.05.2018
 
//#define BLYNK_DEBUG
#define BLYNK_PRINT Serial
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#define CLOUD  // comment out for local server

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
 
#define OLED_RESET 0  // GPIO0
Adafruit_SSD1306 display(OLED_RESET);
 
BlynkTimer timer;


char auth[]       = "xxxx";
char ssid[]       = "xxxx";
char pass[]       = "xxxx";
char server[]     = "blynk-cloud.com";    // ip or domain
char myhostname[] = "Energy-Meter-V2.0";  // for OTA and router identification

const int Sensor_Pin = A0;
unsigned int Sensitivity = 185;   // 185mV/A for 5A, 100 mV/A for 20A and 66mV/A for 30A Module
float Vpp = 0; // peak-peak voltage 
float Vrms = 0; // rms voltage
float Irms = 0; // rms current
float Supply_Voltage = 233.0;           // reading from DMM
float Vcc = 5.0;         // ADC reference voltage // voltage at 5V pin 
float power = 0;         // power in watt              
float Wh =0 ;             // Energy in kWh
unsigned long last_time =0;
unsigned long current_time =0;
unsigned long interval = 100;
unsigned int calibration = 100;  // V2 slider calibrates this
unsigned int pF = 85;           // Power Factor default 95
float bill_amount = 0;   // 30 day cost as present energy usage incl approx PF 
unsigned int energyTariff = 8.0; // Energy cost in INR per unit (kWh)

void getACS712() {  // for AC
  Vpp = getVPP();
  Vrms = (Vpp/2.0) *0.707; 
  Vrms = Vrms - (calibration / 10000.0);     // calibtrate to zero with slider
  Irms = (Vrms * 1000)/Sensitivity ;
  if((Irms > -0.015) && (Irms < 0.008)){  // remove low end chatter
    Irms = 0.0;
  }
  power= (Supply_Voltage * Irms) * (pF / 100.0); 
  last_time = current_time;
  current_time = millis();    
  Wh = Wh+  power *(( current_time -last_time) /3600000.0) ; // calculating energy in Watt-Hour
  bill_amount = Wh * (energyTariff/1000);
  Serial.print("Irms:  "); 
  Serial.print(String(Irms, 3));
  Serial.println(" A");
  Serial.print("Power: ");   
  Serial.print(String(power, 3)); 
  Serial.println(" W"); 
  Serial.print("  Bill Amount: INR"); 
  Serial.println(String(bill_amount, 2));
  Blynk.virtualWrite(V0, String (Wh));  // gauge 
  Blynk.virtualWrite(V1, String(bill_amount, 2));
  Blynk.virtualWrite(V2, String(power,2));
  Blynk.virtualWrite(V3, String(Irms, 3));    
}

float getVPP()
{
  float result; 
  int readValue;                
  int maxValue = 0;             
  int minValue = 1024;          
  uint32_t start_time = millis();
  while((millis()-start_time) < 950) //read every 0.95 Sec
  {
     readValue = analogRead(Sensor_Pin);    
     if (readValue > maxValue) 
     {         
         maxValue = readValue; 
     }
     if (readValue < minValue) 
     {          
         minValue = readValue;
     }
  } 
   result = ((maxValue - minValue) * Vcc) / 1024.0;  
   return result;
 }
 
BLYNK_WRITE(V4) {  // calibration slider 50 to 200
    calibration = param.asInt();
    }
BLYNK_WRITE(V5) {  // set supply voltage slider 70 to 260
    Supply_Voltage = param.asInt();
    }
BLYNK_WRITE(V6) {  // PF slider 60 to 100 i.e 0.60 to 1.00, default 85
   pF = param.asInt();
   }
BLYNK_WRITE(V7) {  // Energy tariff slider 1 to 20, default 8 (Rs.8.0 / kWh)
    energyTariff = param.asInt();
}
BLYNK_WRITE(V8) {  // set 5, 20 or 30A ACS712 sensor with menu
    switch (param.asInt())
    {
      case 1: {       // 5A
      Sensitivity = 185;
        break;
      }
      case 2: {       // 20A
       Sensitivity  = 100;
        break;
      }
      case 3: {       // 30A
       Sensitivity  = 66;
        break;
      }
      default: {      // 5A
      Sensitivity  = 185;
      }
    }
}
void displaydata() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(10, 3);
  display.println("Pow  : "); 
  display.setCursor(50, 3);  
  display.println(power);
  display.setCursor(85, 3);
  display.println("W"); 
  
  display.setCursor(10, 13);
  display.println("Ener : ");  
  display.setCursor(50, 13);
  display.println(Wh);
  display.setCursor(85, 13);
  display.println("Wh");

  display.setCursor(10, 23);
  display.println("Bill : "); 
  display.setCursor(50, 23);
  display.println(bill_amount);
  display.setCursor(85, 23);
  display.println("INR");
  display.display();
}


void setup() {
  display.begin();   
  WiFi.hostname(myhostname);
  Serial.begin(115200); 
  Serial.println("\n Rebooted");
  WiFi.mode(WIFI_STA);
  #ifdef CLOUD
    Blynk.begin(auth, ssid, pass);
  #else
    Blynk.begin(auth, ssid, pass, server);
  #endif
  while (Blynk.connect() == false) {}
  ArduinoOTA.setHostname(myhostname);
  ArduinoOTA.begin();
  timer.setInterval(2000L, getACS712); // get data every 2s
}

BLYNK_CONNECTED(){
  Blynk.syncAll();  
}

void loop() {

  displaydata();
  Blynk.run();
  ArduinoOTA.handle();
  timer.run();
 
}void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
