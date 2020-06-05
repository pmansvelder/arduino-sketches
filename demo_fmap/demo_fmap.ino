float fmap(float value, float min, float max, float tmin, float tmax) {
  return (value - min) * (tmax - tmin) / (max - min) + tmin;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  for (int i = 0 ; i < 1024 ; i++) {
    Serial.println("i = "+String(i)+" fmap(i) = "+String(fmap(i,0,1023,0,100)));
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
