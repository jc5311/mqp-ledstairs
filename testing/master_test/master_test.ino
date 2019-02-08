#include <NeoSWSerial.h>

#define MAX_LED_BARS 255
#define DATA_LENGTH 4
#define TRUE 1
#define FALSE 0
uint8_t ssrx_pin = 8;
uint8_t sstx_pin = 7;
uint8_t registering_led_bars = TRUE;
int ledBar[255];
uint8_t num_bars = 0;

NeoSWSerial ss(ssrx_pin, sstx_pin);

char sSerialRx(void);

void setup()
{
    //start and write with NeoSWSerial
    ss.begin(9600);

    //begin setup for led bars
    uint8_t bar_setup_index = 0;
    int8_t bar_id = 0;
    while(registering_led_bars && (bar_setup_index < MAX_LED_BARS))
    {
        //dont escape until we've learned about some led bars we cannot do anything
        //unless we are aware of how many are connected
        if (ss.available() > 0)
        {
            //check if an id was received
            if ((bar_id = sSerialRx()) > 0) //no bar should have id of 0
            {
                ledBar[bar_setup_index] = bar_id;
                bar_setup_index++;
            }

            //check if no more ids are incoming and stop trying to register
            else if (bar_id == 0)
            {
                registering_led_bars = FALSE;
            }
        }
    }
    num_bars = bar_setup_index + 1; //Use latest index to form number of bars registered

    //do other setup stuff
}

void loop()
{
    for (int i = 0; i < num_bars; i++)
    {
        ss.println("addr= ");
        ss.println(ledBar[i], HEX);
    }
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
