#define LED_PIN 13
#define LED_PWM_PIN 11

#define EN_PIN_A 12
#define EN_PIN_B 8
#define BTN_PIN 4


unsigned char encoder_A;
unsigned char encoder_B;
unsigned char last_encoder_A;
unsigned char last_encoder_B;
int led_power = 0;
int power_step = 1;
long loop_time;
long button_time;
bool last_button_state;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(EN_PIN_A, INPUT_PULLUP);
  pinMode(EN_PIN_B, INPUT_PULLUP);
  pinMode(BTN_PIN, INPUT_PULLUP);

  analogWrite(LED_PWM_PIN, 0);

  Serial.begin( 115200 );
  Serial.println( "START" );
}

void loop() {
  long current_time = millis(); //millis() - Returns the number of milliseconds since the Arduino board began running the current program.

  bool btn = digitalRead(BTN_PIN);  //Read state of encoder switch.
  if ( btn != last_button_state && current_time - button_time > 100 ) { //If the state is different from the previous one and since last state change has been at least 100ms.
    if ( btn == LOW && last_button_state ) { //Detect the transitions from HIGH to LOW
      if ( led_power == 255 ) {
        led_power = 0;
      } else {
        led_power = 255;
      }
      analogWrite(LED_PWM_PIN, led_power);
    }
    last_button_state = btn;
    button_time = current_time;
  }

  if ( current_time - loop_time >= 5 ) {
    encoder_A = digitalRead(EN_PIN_A);  //Read encoder pin A
    encoder_B = digitalRead(EN_PIN_B);  //Read encoder pin B

    if ( !encoder_A && last_encoder_A ) {
      if ( encoder_B ) {
        Serial.println( "L: " + String(led_power) );
        led_power = led_power - power_step;
      } else {
        Serial.println( "R: " + String(led_power) );
        led_power = led_power + power_step;
      }

      if ( led_power < 0 ) led_power = 0;
      if ( led_power >= 255 ) led_power = 255;

      if ( led_power >= 0 && led_power <= 10 ) {
        power_step = 1;
      } else if ( led_power > 10 && led_power <= 20 ) {
        power_step = 2;
      } else if ( led_power > 20 && led_power <= 30 ) {
        power_step = 5;
      } else if ( led_power > 30 ) {
        power_step = 10;
      }

      analogWrite(LED_PWM_PIN, led_power);
    }

    last_encoder_A = encoder_A;
    last_encoder_B = encoder_B;

    loop_time = current_time;
  }
}

