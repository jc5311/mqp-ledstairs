
uint8_t sent_own = 0;
uint8_t to_send = 0;
uint8_t received_continue = 0;
uint8_t received_done = 0;

void setup()
{

}

void loop()
{

}

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
  uint8_t rcv_color = 0;
  uint8_t rcv_cont = 0;

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
      Serial.println("Light byte found, setting rcv_in_progress to TRUE");
      //acknowledge the start bit and start looking for incoming data
      rcv_in_progress = TRUE;
      rcv_color = TRUE;
    }
    
    //did we receive a continue packet?
    else if (rb == 0xC0)
    {
        rcv_in_progress = TRUE;
        rcv_cont = TRUE;
    }

    //do nothing if we receive data thats not the start bit and if we are not
    //actively seeking additional data. What this will cause is that anytime
    //we receive serial data and the buffer is full, the arduino will sift
    //through the data until a start bit is found in case a packet was properly
    //sent. Otherwise the function should throw an exception.
  
  } //end of while loop

  if (data_available == TRUE && return_buffer[0] == id){
    color_array[0] = return_buffer[1];
    color_array[1] = return_buffer[2];
    color_array[2] = return_buffer[3];
  }
    return;
}

//process serial packet and return the received address
char sSerialRx(void)
{
    Serial.println("Entered serialRcv()");
  //initialize neccesary variables
  uint8_t rcv_in_progress = FALSE;
  uint8_t data_available = FALSE;
  uint8_t loop_index = 0;
  uint8_t data_index = 0;
  uint8_t rb;
  uint8_t rcv_buffer[DATA_LENGTH] = {0, 0, 0, 0};

  //loop so long as serial data is still available
  //and all of it hasn't been collected yet
  while ((ss.available() > 0) && (data_available == FALSE)){
    //capture data right away
    rb = ss.read();

    //are we actively reading data? if so, continue processing it
    if (rcv_in_progress == TRUE){
      //check if the datum is the end byte and close up shop if so
      if (rb == 0xBB){
        ss.println("End byte found, setting data_available to TRUE");
        rcv_in_progress = FALSE;
        data_available = TRUE;
      }

      //otherwise save the data and continue
      else{
        ss.println("rb= ");
        ss.println(rb, HEX);
        ss.println("data_index= ");
        ss.println(data_index, DEC);
        rcv_buffer[data_index] = rb;
        data_index++;
      }
    }

    //if not then is the byte we just read the start byte of an id message?
    else if (rb == 0xCC){
      ss.println("Start byte found, setting rcv_in_progress to TRUE");
      //acknowledge the start bit and start looking for incoming data
      rcv_in_progress = TRUE;
    }

    //do nothing if we receive data thats not the start bit and if we are not
    //actively seeking additional data. What this will cause is that anytime
    //we receive serial data and the buffer is full, the arduino will sift
    //through the data until a start bit is found in case a packet was properly
    //sent. Otherwise the function should throw an exception.
  
  } //end of while loop

  if (data_available == TRUE)
  {
      return rcv_buffer[0]; //return the id last received
  }
    return 0; //zero means no id found
}

void sSerialTx(uint8_t addr_to_send)
{
    //tx a packet similar to how we do with led bars
    //except put an addr in the message


    //0xAA signals start of lighting packet
    //0xCC signals addr packet
    //0xBB signals end of any packet
    uint8_t buffer[] = {0xCC, 0x00, 0x00, 0x00, 0x00, 0xBB};

    //package argument details into buffer
    buffer[1] = addr_to_send;
    buffer[2] = 0;//red * light_dimness;
    buffer[3]= 0;//green * light_dimness;
    buffer[4] = 0;//blue * light_dimness;

    //Write buffer to serial
    ss.write(buffer, sizeof(buffer));
}