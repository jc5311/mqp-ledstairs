#define PACKET_LENGTH 6
#define COMMON_ANODE 1
#define COMMON_CATHODE 0
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
 setBarColor(0xAB, 232, 12, 122, COMMON_ANODE);
 delay(250);
 setBarColor(0xBC, 0, 255, 0, COMMON_ANODE);
 delay(250);
 setBarColor(0xCD, 0, 0, 255, COMMON_ANODE);
 delay(250);
 setBarColor(0xAB, 0, 0, 0, COMMON_ANODE);
 delay(250);
 setBarColor(0xBC, 0, 0, 0, COMMON_ANODE);
 delay(250);
 setBarColor(0xCD, 0, 0, 0, COMMON_ANODE);
 delay(250);
}

//wrapper function to send color message
void setBarColor(uint8_t bar_addr, uint8_t red, uint8_t green, uint8_t blue, uint8_t cnxn_type){
  uint8_t buffer[] = {0xAA, 0x00, 0x00, 0x00, 0x00, 0xBB};

  //package argument details into buffer
  //if leds are common cathode pass color values as-is
  if (cnxn_type == COMMON_CATHODE){
    buffer[1] = bar_addr;
    buffer[2] = red;
    buffer[3]= green;
    buffer[4] = blue;
  }
  //if leds are common anode reverse the color values
  else if (cnxn_type == COMMON_ANODE){
    buffer[1] = bar_addr;
    buffer[2] = EIGHTBITMAX - red;
    buffer[3]= EIGHTBITMAX - green;
    buffer[4] = EIGHTBITMAX - blue;
  }

  //Write buffer to serial
  Serial.write(buffer, sizeof(buffer));
}
