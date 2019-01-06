 #define BUFFER_LENGTH 64
 #define PACKET_LENGTH 6
 #define DATA_LENGTH 4

  //setup some global vars
  char id = '3';
  int state = 0;
  uint8_t color_array[3] = {0, 0, 0}; //initialize to white
  boolean data_available = false;
  uint8_t return_buffer[DATA_LENGTH];



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
    serialRcv();
    Serial.println("---Color array values as seen by loop()---");
    Serial.println(color_array[0], DEC);
    Serial.println(color_array[1], DEC);
    Serial.println(color_array[2], DEC);
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
  
  Serial.println("---Color array values as seen by readColorMessage()---");
  Serial.println(color_array[0], DEC);
  Serial.println(color_array[1], DEC);
  Serial.println(color_array[2], DEC);
  return;
}

//receive data from master and return its contents
void serialRcv(void){
  //initialize neccesary variables
  boolean rcv_in_progress = false;
  uint8_t loop_index = 0;
  uint8_t data_index = 0;
  uint8_t rb;

  //loop so long as serial data is still available
  //and all of it hasn't been collected yet
  while ((Serial.available() > 0) && (data_available == false)){
    //capture data right away
    rb = Serial.read();
    Serial.println("---rb value read---");
    Serial.println(rb, HEX);


    //are we actively reading data? if so, continue processing it
    if (rcv_in_progress){
      //check if the datum is the end byte and close up shop if so
      if (rb == 0xBB){
        rcv_in_progress = false;
        data_available = true;
      }

      //otherwise save the data and continue
      else{
        return_buffer[data_index] = rb;
        data_index++;
      }
    }

    //if not then is the byte we just read the start byte?
    else if (rb == 0xAA){
      //acknowledge the start bit and start looking for incoming data
      rcv_in_progress = true;
    }

    //do nothing if we receive data thats not the start bit and if we are not
    //actively seeking additional data. What this will cause is that anytime
    //we receive serial data and the buffer is full, the arduino will sift
    //through the data until a start bit is found in case a packet was properly
    //sent. Otherwise the function should throw an exception.

  }

  if (data_available){
    color_array[0] = return_buffer[1];
    color_array[1] = return_buffer[2];
    color_array[2] = return_buffer[3];
    return;
  }
  else{
    return;
  }
}