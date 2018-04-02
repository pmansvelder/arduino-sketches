#include <Servo.h>

// Originele sketch 28-7-2017 door HansQ
// Aangepast 1-2-2018 door Peter Mansvelder
// ToDo: OMHOOG & OMLAAG zijn omgewisseld!!!

// DEBUG flag
boolean DEBUG;

// I/O definities
int LED;
int SERVO;
int INGANG;
int UITGANG;
double VOLTAGE;

/// Mogelijke toestanden
int INITIALISATIE;
int GAAT_OMHOOG;
int STAAT_OMHOOG;
int GAAT_OMLAAG;
int STAAT_OMLAAG;

// Diverse constantes
int HOEK_OMLAAG;
int HOEK_OMHOOG;
int SERVO_MINIMUM;
int SERVO_MAXIMUM;

// Tijd constantes
int TIJD_STAP;
int TIJD_KNIPPER;
int TIJD_DREMPEL;
int TIJD_MAXIMUM;

// Globale variabelen
Servo servo;
int toestand;
int tijd;

// Converteer de beweeg-tijd omhoog/omlaag naar een servo hoek in milliseconden.
int tijd2us(int tijd)
{
  double stap = (0.0 + HOEK_OMHOOG - HOEK_OMLAAG) / TIJD_MAXIMUM;
  double hoek = HOEK_OMLAAG + stap * tijd;
  int us = SERVO_MINIMUM + hoek / 180 * (SERVO_MAXIMUM - SERVO_MINIMUM);

  // Voorkom dat de server gaat staan "trillen" tegen z'n eindpunt...
  if (us < SERVO_MINIMUM)
    us = SERVO_MINIMUM;
  if (us > SERVO_MAXIMUM)
    us = SERVO_MAXIMUM;

  // Indien nodig, schrijf een DEBUG regel en return de berekende waarde.
  if (DEBUG) {
    Serial.print("DEBUG: ");
    Serial.print("hoek = ");
    Serial.print(hoek, DEC);
    Serial.print(", us = ");
    Serial.println(us);
  }
  return us;
}

void setup() 
{ 
  DEBUG = false;
  
  // I/O definities
  LED = 13;  // Standaard board LED op pin 13
  SERVO = 3;  // Servo op pin 3
  INGANG = 0;  // Analoge ingang op pin A0
  VOLTAGE = 0.5;  // Grens voltage voor ingang

  // Er zijn 5 verschillende toestanden:
  INITIALISATIE = 0;  
  GAAT_OMHOOG = 1;  
  STAAT_OMHOOG = 2;
  GAAT_OMLAAG = 3;  
  STAAT_OMLAAG = 4;  

  // De begin- en eind-hoek van de servo
  HOEK_OMLAAG = 100;  // Tussen de 0 en de 180, mag hoger zijn dan HOEK_OMLAAG
  HOEK_OMHOOG = 150;  // Tussen de 0 en de 180, mag lager zijn dan HOEK_OMLAAG
  SERVO_MINIMUM = 544; // Servo minimum pulse breedte, bij 0 graden (zie Servo.h)
  SERVO_MAXIMUM = 2400; // Servo maximum pulse breedte, bij 180 graden (zie Servo.h)

  // Timing
  TIJD_STAP = 10;  // Tijdstap in milliseconden, aanbevolen is 10ms vanwege de servo
  TIJD_DREMPEL = 100; // Drempel voor acceptatie van ingang verandering, groter dan TIJD_STAP
  TIJD_KNIPPER = 1000; // Knipper tijd voor de LED, initialisatie duurt 3 x TIJD_KNIPPER
  TIJD_MAXIMUM = 25000; // Tijdsduur voor het "omhoog" en "omlaag" bewegen

  // I/O setup
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);  // LED initieel uit
  analogReference(DEFAULT);  // between 0V and 5V
  servo.attach(SERVO);
  servo.writeMicroseconds(tijd2us(TIJD_MAXIMUM));  // Initialiseer de servo

  // De begin toestand is de initialisatie, om de servo tijd te gunnen en zet de tijd op 0.
  toestand = INITIALISATIE;  
  tijd = 0;
  if (DEBUG)
    Serial.begin(9600);
} 

void loop() 
{

  // Voer de initialisatie uit, wacht 3 x TIJD_KNIPPER...
  if (toestand == INITIALISATIE) {
    if (DEBUG)
      Serial.println("INITIALISATIE");
      
    // Laat het LEDje op het Arduino board knipperen.
    if (tijd % TIJD_KNIPPER == 0)
      digitalWrite(LED, LOW);
    if (tijd % TIJD_KNIPPER == (TIJD_KNIPPER / 2))
      digitalWrite(LED, HIGH);

    // Als de LED 3 keer heeft geknipperd, ga naar de "staat omlaag" toestand en zet de tijd op 0.
    if (tijd == (TIJD_KNIPPER * 3)) {
      digitalWrite(LED, LOW);  // Zet ook de LED op "omlaag"
      toestand = STAAT_OMHOOG;
      tijd = 0;
    }
  }

  // Als aan het omhoog bewegen zijn, bereken dan steeds de hoek en verzet de servo omhoog.
  if (toestand == GAAT_OMHOOG) {
    Serial.println("GAAT_OMHOOG");
    int us = tijd2us(tijd);
    servo.writeMicroseconds(us);  // nauwkeuriger dan servo.write(hoek)     

    // Als de maximum tijd verstreken is, verander naar de "staat omhoog" toestand en zet de tijd op 0.
    if (tijd == TIJD_MAXIMUM) {
      toestand = STAAT_OMHOOG;
      tijd = 0;
    }
  }

  // Stabiele toestand "omhoog", kijk of de ingang veranderd en de toestand naar "omlaag" moet.
  if (toestand == STAAT_OMHOOG) {
    Serial.println("STAAT_OMHOOG");
    if (analogRead(INGANG) > VOLTAGE)  // Meer dan het grens voltage op de analoge ingang?
      tijd = 0;
      
   // Als de de ingang lang genoeg "laag" is, moeten we naar de "omlaag bewegen" toestand en de tijd op 0.
   if (tijd == TIJD_DREMPEL) {
      digitalWrite(LED, LOW);  // Zet ook de LED op "omlaag"
      toestand = GAAT_OMLAAG;
      tijd = 0;
    }
  }

  // Als aan het omlaag bewegen zijn, bereken dan steeds de hoek en verzet de servo omlaag.
  if (toestand == GAAT_OMLAAG) {
    Serial.println("GAAT_OMLAAG");
    int us = tijd2us(TIJD_MAXIMUM - tijd);
    servo.writeMicroseconds(us);  // nauwkeuriger dan servo.write(hoek)     

    // Als de maximum tijd verstreken is, verander naar de "staat omlaag" toestand en zet de tijd op 0.
    if (tijd == TIJD_MAXIMUM) {
      toestand = STAAT_OMLAAG;
      tijd = 0;
    }
  }

  // Stabiele toestand "omlaag", kijk of de ingang veranderd en de toestand naar "omhoog" moet.
  if (toestand == STAAT_OMLAAG) {
    Serial.println(analogRead(INGANG));
    if (analogRead(INGANG) < VOLTAGE)  // Minder dan het grens voltage op de analoge ingang?
      tijd = 0;
      
    // Als de de ingang lang genoeg "hoog" is, moeten we naar de "omhoog bewegen" toestand en de tijd op 0.
    if (tijd == TIJD_DREMPEL) {
      digitalWrite(LED, HIGH);  // Zet ook de LED op "omhoog"
      toestand = GAAT_OMHOOG;
      tijd = 0;
    }
  }

  // Wacht een tijdstap en hoog de tijd teller op.
  delay(TIJD_STAP); 
  tijd += TIJD_STAP;
}


