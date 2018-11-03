void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("<Arduino is ready>");
}

char rx_byte = "";
String rx_text = "";
long DefaultValue = 999;

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    rx_byte = Serial.read();       // get the character
    rx_text += rx_byte;
  } // end: if (Serial.available() > 0)
  if (rx_byte == '\n') {
    Serial.println("Final Text = " + rx_text);
    long value = rx_text.substring(2).toInt();
    if (value == 0) value = DefaultValue;
    Serial.print("Value = ");
    Serial.println(value);
    rx_byte = "";
    rx_text = "";
  }
}
