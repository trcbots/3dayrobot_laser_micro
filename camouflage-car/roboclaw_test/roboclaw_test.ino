//Roboclaw simple serial example.  Set mode to 6.  Option to 4(38400 bps)


void setup() {
//  mySerial.begin(38400);
  Serial3.begin(38400);
}

void loop() {
  delay(1000);
//  Serial3.write(128);
//  delay(1000);
//  Serial3.write(192);
//  delay(1000);
//  Serial3.write(255);
//  delay(3000);
//  Serial3.write(192);

  for(int i = 0; i < 1000; i++)
  {
    Serial3.write(255);
  }
}
