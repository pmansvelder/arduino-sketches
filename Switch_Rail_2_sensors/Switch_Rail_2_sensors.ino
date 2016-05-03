volatile long TimerCount = 0;
volatile int val,status;
volatile int RetryDelay;

void setup()
{
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  pinMode(2, INPUT);
  pinMode(13, OUTPUT);
  pinMode(3, INPUT);
  digitalWrite(3, HIGH);    // Enable pullup resistor
  digitalWrite(2, HIGH);    // Enable pullup resistor
  digitalWrite(13, HIGH);

}
//
void loop(void)
{
  //    Serial.print("Aantal interrupts: ");
  //    Serial.println(TimerCount);
  //    val = digitalRead(3);
  //    Serial.println(digitalRead(3));
  //  Serial.println("Start detectie...");
  digitalWrite(13, LOW);    // Zet spanning op rail
  if (digitalRead(3) == LOW) {  // positief signaal 
    //    Serial.println("Positief signaal gedetecteerd");
    if (digitalRead(2) == HIGH) { // geen signaal op rail
      digitalWrite(13, HIGH);    // Haal spanning van rail
      Serial.println("Kortsluiting gedetecteerd");
      RetryDelay = 2500;
    }
    else {
      //      Serial.println("Kortsluiting opgeheven");
    }
  }
  TimerCount = 0;
  delay(RetryDelay);

}




