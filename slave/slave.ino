 #define BUFFER_LENGTH 64
 #define PACKET_LENGTH 6
 #define DATA_LENGTH 4
 #define TRUE 1
 #define FALSE 0

  //setup some global vars
  char id = '3';
  int state = 0;
  uint8_t color_array[3] = {0, 0, 0}; //initialize to white
  boolean data_available = false;
  uint8_t return_buffer[DATA_LENGTH] = {255, 255, 255, 255};



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
    serialRcv();
    Serial.println("---Color array values as seen by loop()---");
    Serial.println(color_array[0], DEC);
    Serial.println(color_array[1], DEC);
    Serial.println(color_array[2], DEC);
    analogWrite(9, color_array[0]);
    analogWrite(10, color_array[1]);
    analogWrite(11, color_array[2]);
  }//end of serial available check

  
} //end of loopPACKET_LENGTH

//receive data from master and update color array
void serialRcv(void){
  //initialize neccesary variables
  boolean rcv_in_progress = false;
  uint8_t loop_index = 0;
  uint8_t data_index = 0;
  uint8_t rb;

  //loop so long as serial data is still available
  //and all of it hasn't been collected yet
  while ((Serial.available() > 0) || (data_available == FALSE)){
    //capture data right away
    rb = Serial.read();

    //are we actively reading data? if so, continue processing it
    if (rcv_in_progress){
      Serial.println("---------");
      Serial.println("Entered rcv_in_progress block");
      //check if the datum is the end byte and close up shop if so
      if (rb == 0xBB){
        rcv_in_progress = FALSE;
        data_available = TRUE;
      }

      //otherwise save the data and continue
      else{
        Serial.println("---------");
        Serial.println("rb as seen by SerialRcv assignment block");
        Serial.println(rb, HEX);
        return_buffer[data_index] = rb;
        data_index++;
      }
    }

    //if not then is the byte we just read the start byte?
    else if (rb == 0xAA){
      //acknowledge the start bit and start looking for incoming data
      rcv_in_progress = TRUE;
    }

    //do nothing if we receive data thats not the start bit and if we are not
    //actively seeking additional data. What this will cause is that anytime
    //we receive serial data and the buffer is full, the arduino will sift
    //through the data until a start bit is found in case a packet was properly
    //sent. Otherwise the function should throw an exception.

  }

  /*if (data_available == true){*/
    Serial.println("---------");
    Serial.println("return_buffer[1]: ");
    Serial.println(return_buffer[1], HEX);
    Serial.println("return_buffer[2]: ");
    Serial.println(return_buffer[2], HEX);
    Serial.println("return_buffer[3]: ");
    Serial.println(return_buffer[3], HEX);
    color_array[0] = return_buffer[1];
    color_array[1] = return_buffer[2];
    color_array[2] = return_buffer[3];
    /*return;
  }
  else{
    return;
  }*/
}