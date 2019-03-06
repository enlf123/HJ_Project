// reference : https://blog.naver.com/i2asys/221257469969

int sensorValue = 0;
int sensorValue_old = 0;
int cnt = 0;
double ang = 0.0;
double preset[5];
double avg_preset = 0;
// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);
  // initialization for removing preset
  for(int i=0;i<5;i++){
  sensorValue = analogRead(A0);
  if (sensorValue-sensorValue_old > 500) cnt = cnt - 1;
  else if (sensorValue-sensorValue_old < -500) cnt = cnt + 1;
  ang = (double)cnt*360.0 + (double)sensorValue*360.0/1023.0;
  sensorValue_old = sensorValue; //이전 센서 값을 지금 읽은 값으로 업데이트
  preset[i] =ang;
  }
  avg_preset = (preset[0]+preset[1]+preset[2]+preset[3]+preset[4])/5;
}

// the loop routine runs over and over again forever:
void loop() {
//   read the input on analog pin 0:
  sensorValue = analogRead(A0);
  if (sensorValue-sensorValue_old > 500) cnt = cnt - 1;
  else if (sensorValue-sensorValue_old < -500) cnt = cnt + 1;
  ang = (double)cnt*360.0 + (double)sensorValue*360.0/1023.0;

  sensorValue_old = sensorValue; //이전 센서 값을 지금 읽은 값으로 업데이트
 // print out the ang:
  Serial.println(ang-avg_preset);
  delay(50);
}
