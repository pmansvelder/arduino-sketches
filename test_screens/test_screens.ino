// sketch to test control connection for sunscreens
// to control the shades, the following connection is used:
// 1 Earth
// 2 Common
// 3 Phase - direction 1
// 4 Phase - direction 2

const byte relayPin1 = 5; // Direction
const byte relayPin2 = 6; // Power
const int MoveTime = 5000; // Time to move screens
const byte TestPin = 13;

void setup() {
  // put your setup code here, to run once:
  pinMode(relayPin1, OUTPUT);
  pinMode(relayPin2, OUTPUT);
  pinMode(TestPin, OUTPUT);
}

void MoveUp() {
  digitalWrite(relayPin1, LOW);
  digitalWrite(relayPin2, LOW);
  delay(MoveTime);
  digitalWrite(relayPin2, HIGH);
}

void MoveDown() {
  digitalWrite(relayPin1, HIGH);
  digitalWrite(relayPin2, LOW);
  delay(MoveTime);
  digitalWrite(relayPin2, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(TestPin, HIGH);
  MoveUp();
  digitalWrite(TestPin, LOW);
  delay(1000);
  digitalWrite(TestPin, HIGH);
  MoveDown();
  digitalWrite(TestPin, LOW);
  delay(1000);
}
