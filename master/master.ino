void setup() {
  // put your setup code here, to run once:

  //setup serial connection
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  //loop and send a message to everyone
  Serial.write('1');
  delay(1000);
  Serial.write('2');
  delay(1000);
  Serial.write('3');
  delay(1000);
}
