/**
 * Author(s):    Juan Caraballo
 * Created:   1/16/19
 * Copyright 2019, Juan Caraballo, All rights reserved.
 * 
 * Description: This Arduino file implements the software for a master mcu device
 * within a led stair system. The master serves as the primary controller for an
 * array of led bars and communicates how and when animations should (should not)
 * occur.
 * 
 * Usage: After flashing this software onto a Arduino it will function
 * autonomously. Animations will toggle through in time unless an external
 * receiver trip is signaled; after which the system will wait 5 seconds before
 * recommencing animation.
 * 
 * (( Note to self
 * Add notes for pinouts
 * Add notes for animation/system timing
 * ))
 */

//includes
#include <Arduino_FreeRTOS.h> //RTOS library for scheduling tasks

//defines
#define PACKET_LENGTH 6
#define LED_BAR_COUNT 3
#define NORMAL_RANGE 1
#define FLIPPED_RANGE 0
#define EIGHTBITMAX 255
#define BAUD_RATE 115200 //master and slave mcu bauds MUST match
#define COOLDOWN_PERIOD 5 //timer cooldown period (in seconds)

//globals
uint8_t interrupt_pin = 2; //only p2 and p3 can be used for interrupt on nano
uint8_t led_bar[LED_BAR_COUNT]; //array to hold led bar addresses
uint8_t timer_done = 0;
uint8_t timer_count = 0;

//prototypes
void setBarColor(uint8_t bar_addr, uint8_t red, 
                 uint8_t green, uint8_t blue, uint8_t range_type);
void disableLedBars(uint8_t led_bar_count, uint8_t led_bar[]);
void rcvrISR(void);
void cooldownTimer(void);
//rtos tasks
void TaskAnimate(void *pvParameters);
void TaskAnimationDisable(void *pvParameters);

//rtos semaphores
SemaphoreHandle_t xLedDisableSemaphore;

void setup() {
  //initialize interrupt pin and configure pullup resistor
  pinMode(interrupt_pin, INPUT_PULLUP);
  //attach interrupt to the interrupt pin and configure trigger on falling edge
  attachInterrupt(digitalPinToInterrupt(interrupt_pin), rcvrISR, FALLING);

  //setup serial connection
  Serial.begin(BAUD_RATE);
  while(!Serial); //wait for serial port to connect

  //fill led bar array with led bar addresses
  led_bar[0] = 0xAB;
  led_bar[1] = 0xBC;
  led_bar[2] = 0xCD;

  //configure semaphore
  if (xLedDisabeleSemaphore == NULL){ //confirm semaphore wasn't already made
    xLedDisableSemaphore = xSemaphoreCreateMutex(); //mutex semaphore
  }

  //configure RTOS tasks
  xTaskCreate(
    TaskAnimate,
    (const portCHAR *)"Toggle Animations",
    128, //stack size
    NULL,
    1, //priority
    NULL
  );

  xTaskCreate(
    TaskAnimationDisable,
    (const portCHAR *)"Disable Animations",
    128, //stack size
    NULL,
    2, //priority
    NULL
  );
}

void loop() {
  /*
  * Since we are using an RTOS all work to be done is placed into tasks.
  * Therefore nothing needs to go here. Unless we needed to do some background
  * work when no other high priority tasks were working we could do that here.
  * Otherwise we can conserve energy by sleeping the MCU whenever we enter this
  * loop.
  */
}

//interrupt service routine for timer 0
ISR (TIMER0_OVF_vect){
  timer_count++;

  if (timer_count == COOLDOWN_PERIOD){
    timer_done = 1; //signal that COOLDOWN_PERIOD seconds have passed
    TCCR0B = (0 << CS00) | (0 << CS00) | (0 << CS00); //disable timer
    TCNT0 = 0x00; //clear counter
  }
}

void setBarColor(uint8_t bar_addr, uint8_t red, uint8_t green, uint8_t blue, uint8_t range_type){
  //pend semaphore
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

  //post semaphore
} //end of setBarColor()

void disableLedBars(uint8_t led_bar_count, uint8_t led_bar[]){
  //loop through the number of bars and send a message to turn off
  for (int i = 0; i < led_bar_count; i++){
    setBarColor(led_bar[i], 0, 0, 0, NORMAL_RANGE);
  }
} //end of disableLedBars()

//function to trigger animation disabling and 5 second timeout
//on receiver interrupt
void rcvrISR(void){
  //post led_disable_semaphore

} //end of rcvrISR()

//task to toggle through and execute animations
void TaskAnimate(void *pvParameters){
  (void) pvParameters;

  while(1){
    //loop and send animation messages to everyone
    setBarColor(led_bar[0], 232, 12, 122, NORMAL_RANGE);
    delay(250);
    setBarColor(led_bar[1], 0, 255, 0, NORMAL_RANGE);
    delay(250);
    setBarColor(led_bar[2], 0, 0, 255, NORMAL_RANGE);
    delay(250);
    setBarColor(led_bar[0], 0, 0, 0, NORMAL_RANGE);
    delay(250);
    setBarColor(led_bar[1], 0, 0, 0, NORMAL_RANGE);
    delay(250);
    setBarColor(led_bar[2], 0, 0, 0, NORMAL_RANGE);
    delay(250);
  }
}

//disable animations and wait until signal to animate is given
void TaskAnimationDisable(void *pvParameters){
  (void) pvParameters;
  
  //pend on led_disable_semaphore

  //loop through led bars and send message to disable animation
  disableLedBars(LED_BAR_COUNT, led_bar);
  
  //delay COOLDOWN_PERIOD seconds
  cooldownTimer();
}

void cooldownTimer(void){
  //start timer0
  TCCR0B = 1 << CS00; //timer clock = I/O clock
  TIFR0 = 1 << TOV0; //clear the overflow flag
  TIMSK0 = 1 << TOIE0; //enable timer interrupts
  TCNT0 = 0x00; //clear timer0 counter

  //loop until timer complete
  while (!timer_done);

  timer_done = 0;
  return;
}