#define PACKET_LENGTH 6

void setup() {
  // put your setup code here, to run once:

  //setup serial connection
  Serial.begin(115200);
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:

 //loop and send a message to everyone
 setBarColor(0xAB, 232, 12, 122);
 delay(250);
 setBarColor(0xBC, 0, 255, 0);
 delay(250);
 setBarColor(0xCD, 0, 0, 255);
 delay(250);
 setBarColor(0xAB, 0, 0, 0);
 delay(250);
 setBarColor(0xBC, 0, 0, 0);
 delay(250);
 setBarColor(0xCD, 0, 0, 0);
 delay(250);
}

//wrapper function to send color message
void setBarColor(uint8_t bar_addr, uint8_t red, uint8_t green, uint8_t blue){
  //package argument details into buffer
  uint8_t buffer[] = {0xAA, 0x00, 0x00, 0x00, 0x00, 0xBB};
  buffer[1] = bar_addr;
  buffer[2] = red;
  buffer[3]= green;
  buffer[4] = blue;

  //Write buffer
  Serial.write(buffer, sizeof(buffer));
}
