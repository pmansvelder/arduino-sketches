int column, row;

void setup() {
  // put your setup code here, to run once:
pinMode(0,OUTPUT);
pinMode(1,OUTPUT);
pinMode(2,OUTPUT);
pinMode(3,OUTPUT);
pinMode(4,OUTPUT);
pinMode(5,OUTPUT);
pinMode(6,OUTPUT);
pinMode(7,OUTPUT);
pinMode(8,OUTPUT);
pinMode(9,OUTPUT);
pinMode(10,OUTPUT);
pinMode(11,OUTPUT);
pinMode(12,OUTPUT);
pinMode(13,OUTPUT);
pinMode(14,OUTPUT);
pinMode(15,OUTPUT);
pinMode(16,OUTPUT);
pinMode(17,OUTPUT);
pinMode(18,OUTPUT);
pinMode(19,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
for(int row=16; row<=19; row++){
  digitalWrite(row, HIGH);
  for(int column=0; column<=15; column++){
      digitalWrite(column, HIGH);
      delay(350);
      digitalWrite(column, LOW);
      }
  digitalWrite(row, LOW);
  }
}
