 #define PACKET_LENGTH 6

  //setup some global vars
  char id = '3';
  int state = 0;
  uint8_t color_array[3] = {255, 255, 255}; //initialize to white

void setup() {
  //configure and initialize gpio 6,7, and 8
  pinMode(6, OUTPUT);
  analogWrite(6, color_array[0]);
  pinMode(7, OUTPUT);
  analogWrite(7, color_array[1]);
  pinMode(8, OUTPUT);
  analogWrite(8, color_array[2]);
  
  //begin serial device
  Serial.begin(9600);
}

void loop() {  
  //if serial device is available, read in the data
  if (Serial.available() > 0){
    Serial.println("Entered Serial comm.");
    analogWrite(6, 0);
    analogWrite(7, 0);
    analogWrite(8, 0);
    readColorMessage(color_array);
    analogWrite(6, color_array[0]);
    analogWrite(7, color_array[1]);
    analogWrite(8, color_array[2]);
  }//end of serial available check

  
} //end of loop

//function to read in received byte array sent by master
void readColorMessage(uint8_t* color_array){
  uint8_t buffer[6];
  int i = 0;
  //pull in the first 6 bytes and organize them into the collor array
  for (i = 0; i < PACKET_LENGTH; i ++){
    //if we just started reading, check for the 0xAA start byte
    if (i == 0){
      uint8_t byte_read = Serial.read();
      
      //if we don't see the 0xAA byte, something is wrong, return
      if (byte_read != 0xAA){
        return;
      }

      //we're good, record the 0xAA byte and continue
      else{
        buffer[i] = byte_read;        
      }
    }

    //read the rest of the incoming data
    else{
      buffer[i] = Serial.read();
    }
  } //end of for loop

  //check for the 0xBB end byte, if we get it, then write the buffer
  //to the array given, otherwise return and do nothing
  if (i == PACKET_LENGTH){
    uint8_t byte_read = Serial.read();

    //if the last byte isn't the ending 0xBB byte return and do nothing
    if (byte_read != 0xBB){
      return;
    }
    //else we should be find record it and continute
    else{
      buffer[i] = byte_read;
    }
  }

  //at this point the buffer array should have the same content
  //as was sent to us, so copy it to the color array
  color_array[0] = buffer[2];
  color_array[1] = buffer[3];
  color_array[2] = buffer[4];
  
  Serial.println(color_array[0], DEC);
  Serial.println(color_array[1], DEC);
  Serial.println(color_array[2], DEC);
  return;
}
