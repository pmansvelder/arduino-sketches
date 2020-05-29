byte NumberOfRelays = 3;
const byte RelayPins[] = {6, 11, 12};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  for (int i = 0; i < NumberOfRelays; i++) {
    pinMode(RelayPins[i], OUTPUT);
  }
  pinMode(13, OUTPUT);
  pinMode(9 , OUTPUT);
  digitalWrite(13, HIGH);
  digitalWrite(9, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 0; i < NumberOfRelays; i++) {
    Serial.println("Enabling relay " + String(i));
    digitalWrite(RelayPins[i], HIGH);
    delay(10000);
  }
  for (int i = 0; i < NumberOfRelays; i++) {
    Serial.println("Disabling relay " + String(i));
    digitalWrite(RelayPins[i], LOW);
    delay(10000);
  }
}
