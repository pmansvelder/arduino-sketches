
#include <Servo.h> 

const int max_angle = 200;
const int min_angle = 100;
const int pulses = 20;
const int stap_delay = 12;

int touched;
int counter;
int counter2;
int center_pos;
int right_pos;
int left_pos;

const int left_servo = 3;
const int center_servo = 5;
const int right_servo = 6;
const int left_eye = 4;
const int right_eye = 12;
const int right_feeler = 7;
const int left_feeler = 8;

Servo servo_left;
Servo servo_right;
Servo servo_center;

void setup() {
  // put your setup code here, to run once:
  pinMode(left_eye, OUTPUT);
  digitalWrite(left_eye, HIGH);
  pinMode(right_eye, OUTPUT);
  digitalWrite(right_eye, LOW);
  pinMode(left_feeler, INPUT);
  pinMode(right_feeler, INPUT);
  servo_left.attach(left_servo);
  servo_right.attach(right_servo);
  servo_center.attach(center_servo);
}

void toggle(int output) {
  if (digitalRead(output) == HIGH) {
    digitalWrite(output, LOW);
  }
  else {
    digitalWrite(output, HIGH);
  }
}

void feelers() {
  if (touched == 0) {
    if (digitalRead(left_feeler) == 0) {
      left();
    }
    if (digitalRead(right_feeler) == 0) {
      right();
    }
  }
}

void back() {
  touched = 1;
  digitalWrite(right_eye, HIGH);
  digitalWrite(left_eye, HIGH);
  for (counter2 = 1; counter2 < 5; counter2 += 1);
  {
    center_pos = min_angle ;
    left_pos = max_angle;
    right_pos = max_angle;
    walk();
    center_pos = max_angle;
    left_pos = min_angle;
    right_pos = min_angle;
    walk();
    toggle(right_eye);
    toggle(left_eye);
  }
  toggle(right_eye);
  touched = 0;
}

void right() {
  back();
  digitalWrite(right_eye, HIGH);
  digitalWrite(left_eye, LOW);
  for (counter2 = 1; counter2 < 5; counter2 += 1);
  {
    center_pos = max_angle;
    left_pos = max_angle;
    right_pos = min_angle;
    walk();
    center_pos = min_angle;
    left_pos = min_angle;
    right_pos = max_angle;
    walk();
  }
}

void left() {
  back();
  digitalWrite(right_eye, LOW);
  digitalWrite(left_eye, HIGH);
  for (counter2 = 1; counter2 < 5; counter2 += 1);
  {
    center_pos = min_angle;
    left_pos = max_angle;
    right_pos = min_angle;
    walk();
    center_pos = max_angle;
    left_pos = min_angle;
    right_pos = max_angle;
    walk();
  }
}

void walk() {
  for (counter = 1; counter < pulses; counter+=1)
  {
    servo_center.write(center_pos);
    feelers();
    delay(stap_delay);
  }
  for (counter = 1; counter < pulses; counter+=1)
  {
    servo_right.write(right_pos);
    servo_left.write(left_pos);
    servo_center.write(center_pos);
    feelers();
    delay(stap_delay);
  }
}
 
void loop() {
  // put your main code here, to run repeatedly: 
  toggle(left_eye);
  toggle(right_eye);
  center_pos = max_angle;
  left_pos = max_angle;
  right_pos = max_angle;
  walk();
  toggle(left_eye);
  toggle(right_eye);
  center_pos = min_angle;
  left_pos = min_angle;
  right_pos = min_angle;
  walk();
}
