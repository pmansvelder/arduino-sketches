#define VERSION "1.0"
#define LIBRARY_VERSION "1.1"

String a = VERSION;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print(VERSION);
  Serial.print(" / ");
  Serial.println(LIBRARY_VERSION);
  a += "(";
  a += LIBRARY_VERSION;
  a += ")";
}

void loop() {

  Serial.println(a);
  delay(1000);
  // put your main code here, to run repeatedly:
}
