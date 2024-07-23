int sensorPin[] = {8, 9, 10, 11, 12, 13};
int sensorValue[] = {0, 0, 0, 0, 0, 0};

void setup() {
  Serial.begin(57600);
}

void loop() {
  for(int i = 0; i < 6; i++)
  {
    sensorValue[i] = pulseIn(sensorPin[i],HIGH) - 1490;
    sensorValue[3] = 0;
    if(sensorValue[i] < 30 && sensorValue[i] > -30) sensorValue[i] = 0;
    
    sensorValue[i] = constrain(sensorValue[i], -500, 500);
    
    Serial.print(sensorValue[i]);
    Serial.print(",");
  }
  Serial.println();
}
