void setup() {
  // put your setup code here, to run once:
  pinMode(22, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(13, digitalRead(22));
}
