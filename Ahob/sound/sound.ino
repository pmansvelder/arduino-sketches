void setup() {
  pinMode(6,OUTPUT);
}
void loop() {
  for (int i=150; i<175; i++) {
    analogWrite(6,i);
    delay(8);
  }
  delay(50);
}
