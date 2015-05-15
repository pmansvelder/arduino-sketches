union {
  uint8_t BAR;
  struct {
    uint8_t  r1 : 4; // bit position 0..3
    uint8_t  r2 : 4; // bit positions 4..7
    // total # of bits just needs to add up to the uint8_t size
  } bar;
} foo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  foo.bar.r1 = B1010;
  foo.bar.r2 = B1000;


  Serial.print(F("foo.bar.r1 = 0x"));
  Serial.println(foo.bar.r1, HEX);
  Serial.print(F("foo.bar.r2 = 0x"));
  Serial.println(foo.bar.r2, HEX);

  Serial.print(F("foo.BAR = 0x"));
  Serial.println(foo.BAR, HEX);
}

void loop() {
  // put your main code here, to run repeatedly:

}
