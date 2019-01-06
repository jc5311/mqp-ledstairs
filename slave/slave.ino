 #define BUFFER_LENGTH 64
 #define PACKET_LENGTH 6
 #define DATA_LENGTH 4
 #define TRUE 1
 #define FALSE 0

  //setup some global vars
  uint8_t id = 0xCD;
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
  Serial.begin(115200);
}

void loop() {
  //if serial device is available, read in the data
  if (Serial.available() > 0){
    serialRcv();
    analogWrite(9, color_array[0]);
    analogWrite(10, color_array[1]);
    analogWrite(11, color_array[2]);
  }//end of serial available check

  
} //end of loop

//receive data from master and update color array
void serialRcv(void){
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
        //End byte found, setting data_available to TRUE
        rcv_in_progress = FALSE;
        data_available = TRUE;
      }

      //otherwise save the data and continue
      else{
        return_buffer[data_index] = rb;
        data_index++;
      }
    }

    //if not then is the byte we just read the start byte?
    else if (rb == 0xAA){
      //Start byte found, setting rcv_in_progress to TRUE
      //acknowledge the start bit and start looking for incoming data
      rcv_in_progress = TRUE;
    }
  
  } //end of while loop

  if (data_available == TRUE && return_buffer[0] == id){
    color_array[0] = return_buffer[1];
    color_array[1] = return_buffer[2];
    color_array[2] = return_buffer[3];
  }
    return;
}
