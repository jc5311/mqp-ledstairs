 #define BUFFER_LENGTH 64
 #define PACKET_LENGTH 6
 #define DATA_LENGTH 4
 #define TRUE 1
 #define FALSE 0

  //setup some global vars
  char id = '3';
  int state = 0;
  uint8_t color_array[3] = {0, 0, 0}; //initialize to white



void setup() {
  //configure and initialize gpio 6,7, and 8
  pinMode(9, OUTPUT);
  analogWrite(9, color_array[0]);
  pinMode(10, OUTPUT);
  analogWrite(10, color_array[1]);
  pinMode(11, OUTPUT);
  analogWrite(11, color_array[2]);
  
  //begin serial device
  Serial.begin(9600);
}

void loop() {
  //if serial device is available, read in the data
  if (Serial.available() > 0){
    Serial.println("Calling serialRcv()");
    serialRcv();
    Serial.println("---Color array values as seen by loop()---");
    Serial.println(color_array[0], DEC);
    Serial.println(color_array[1], DEC);
    Serial.println(color_array[2], DEC);
    Serial.println("---Writing to leds---");
    analogWrite(9, color_array[0]);
    analogWrite(10, color_array[1]);
    analogWrite(11, color_array[2]);
    Serial.println("\n\n\n\n\n");
  }//end of serial available check

  
} //end of loop

//receive data from master and update color array
void serialRcv(void){
  Serial.println("Entered serialRcv()");
  //initialize neccesary variables
  uint8_t rcv_in_progress = FALSE;
  uint8_t data_available = FALSE;
  uint8_t loop_index = 0;
  uint8_t data_index = 0;
  uint8_t rb;
  uint8_t return_buffer[DATA_LENGTH] = {255, 255, 255, 255};

  //loop so long as serial data is still available
  //and all of it hasn't been collected yet
  while ((Serial.available() > 0) && (data_available == FALSE)){
    //capture data right away
    rb = Serial.read();

    //are we actively reading data? if so, continue processing it
    if (rcv_in_progress == TRUE){
      //check if the datum is the end byte and close up shop if so
      if (rb == 0xBB){
        Serial.println("End byte found, setting data_available to TRUE");
        rcv_in_progress = FALSE;
        data_available = TRUE;
      }

      //otherwise save the data and continue
      else{
        Serial.println("rb= ");
        Serial.println(rb, HEX);
        Serial.println("data_index= ");
        Serial.println(data_index, DEC);
        return_buffer[data_index] = rb;
        data_index++;
      }
    }

    //if not then is the byte we just read the start byte?
    else if (rb == 0xAA){
      Serial.println("Start byte found, setting rcv_in_progress to TRUE");
      //acknowledge the start bit and start looking for incoming data
      rcv_in_progress = TRUE;
    }

    //do nothing if we receive data thats not the start bit and if we are not
    //actively seeking additional data. What this will cause is that anytime
    //we receive serial data and the buffer is full, the arduino will sift
    //through the data until a start bit is found in case a packet was properly
    //sent. Otherwise the function should throw an exception.
  
  } //end of while loop

  if (data_available == TRUE){
    Serial.print("loop ended and data_available is TRUE, filling color_array");
    Serial.println("return_buffer[1]: ");
    Serial.println(return_buffer[1], HEX);
    Serial.println("return_buffer[2]: ");
    Serial.println(return_buffer[2], HEX);
    Serial.println("return_buffer[3]: ");
    Serial.println(return_buffer[3], HEX);
    color_array[0] = return_buffer[1];
    color_array[1] = return_buffer[2];
    color_array[2] = return_buffer[3];
  }
    return;
}