int LED2 = 13;
int LED1 = 12;
int button = 7;

boolean LED1State = false;
boolean LED2State = false;

long buttonTimer = 0;
long longPressTime = 450;

boolean buttonActive = false;
boolean longPressActive = false;

void setup() {

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(button, INPUT_PULLUP);
  Serial.begin(9600);
}

void loop() {

  if (digitalRead(button) == LOW) {

    if (buttonActive == false) {
      Serial.println("ButtonActive is false");
      buttonActive = true;
      buttonTimer = millis();

    }

    if ((millis() - buttonTimer > longPressTime) && (longPressActive == false)) {
      Serial.println("LongPressActive is true");
      longPressActive = true;
      LED1State = !LED1State;
      digitalWrite(LED1, LED1State);

    }

  } else {

    if (buttonActive == true) {

      if (longPressActive == true) {

        longPressActive = false;

      } else {
        Serial.println("button2 is on");
        LED2State = !LED2State;
        digitalWrite(LED2, LED2State);

      }

      buttonActive = false;

    }

  }

}
