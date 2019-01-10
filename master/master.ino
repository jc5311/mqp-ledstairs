#define PACKET_LENGTH 6
#define NORMAL_RANGE 1
#define FLIPPED_RANGE 0
#define EIGHTBITMAX 255

void setup() {
  // put your setup code here, to run once:

  //setup serial connection
  Serial.begin(115200);
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:

 //loop and send a message to everyone
 setBarColor(0xAB, 232, 12, 122, NORMAL_RANGE);
 delay(250);
 setBarColor(0xBC, 0, 255, 0, NORMAL_RANGE);
 delay(250);
 setBarColor(0xCD, 0, 0, 255, NORMAL_RANGE);
 delay(250);
 setBarColor(0xAB, 0, 0, 0, NORMAL_RANGE);
 delay(250);
 setBarColor(0xBC, 0, 0, 0, NORMAL_RANGE);
 delay(250);
 setBarColor(0xCD, 0, 0, 0, NORMAL_RANGE);
 delay(250);
}

//wrapper function to send color message
void setBarColor(uint8_t bar_addr, uint8_t red, uint8_t green, uint8_t blue, uint8_t range_type){
  uint8_t buffer[] = {0xAA, 0x00, 0x00, 0x00, 0x00, 0xBB};

  //package argument details into buffer
  //if leds are common cathode pass color values as-is
  if (range_type == NORMAL_RANGE){
    buffer[1] = bar_addr;
    buffer[2] = red;
    buffer[3]= green;
    buffer[4] = blue;
  }
  //if leds are common anode reverse the color values
  else if (range_type == FLIPPED_RANGE){
    buffer[1] = bar_addr;
    buffer[2] = EIGHTBITMAX - red;
    buffer[3]= EIGHTBITMAX - green;
    buffer[4] = EIGHTBITMAX - blue;
  }

  //Write buffer to serial
  Serial.write(buffer, sizeof(buffer));
}
