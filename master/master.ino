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
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <task.h>

//defines
#define PACKET_LENGTH 6
#define LED_BAR_COUNT 3
#define NORMAL_RANGE 1
#define FLIPPED_RANGE 0
#define EIGHTBITMAX 255
#define BAUD_RATE 115200 //master and slave mcu bauds MUST match
#define COOLDOWN_PERIOD 5 //timer cooldown period (in seconds)

//globals
uint8_t debug_led = 13;
uint8_t debug_toggle = 0;
uint8_t rcvr_interrupt_pin = 2; //only p2 and p3 can be used for interrupt on nano
uint8_t adc_interrupt_pin = 3; //only p2 and p3 can be used for interrupt on nano
uint8_t led_bar[LED_BAR_COUNT]; //array to hold led bar addresses
volatile uint8_t timer_done = 0;
volatile uint8_t timer_counter = 0;
volatile uint8_t timeout_counter = 0;
uint8_t toggler = 0;
float light_dimness = 1;

//prototypes
void setBarColor(uint8_t bar_addr, uint8_t red, 
                 uint8_t green, uint8_t blue, uint8_t range_type);
void disableLedBars(uint8_t led_bar_count, uint8_t led_bar[]);
void rcvrISR(void);
void cooldownTimer(void);
//rtos tasks
void TaskAnimate(void* pvParameters);
void TaskAnimationDisable(void* pvParameters);
void TaskReadAdcBrightness(void* pvParameters);

//rtos semaphores
SemaphoreHandle_t xLedDisableSemaphore;
SemaphoreHandle_t xAdcUpdateSemaphore;

void setup() {
  //debug led setup
  pinMode(debug_led, OUTPUT);
  digitalWrite(debug_led, LOW);

  //initialize interrupt pin and configure pullup resistor
  pinMode(rcvr_interrupt_pin, INPUT_PULLUP);
  //attach interrupt to the interrupt pin and configure trigger on falling edge
  attachInterrupt(digitalPinToInterrupt(rcvr_interrupt_pin), rcvrISR, FALLING);
  //do the same for the adc for demoing
  pinMode(adc_interrupt_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(adc_interrupt_pin), adcISR, FALLING);

  //setup serial connection
  Serial.begin(BAUD_RATE);
  while(!Serial); //wait for serial port to connect

  //fill led bar array with led bar addresses
  led_bar[0] = 0xAB;
  led_bar[1] = 0xBC;
  led_bar[2] = 0xCD;

  //configure semaphores
  if (xLedDisableSemaphore == NULL){ //confirm semaphore wasn't already made
    xLedDisableSemaphore = xSemaphoreCreateBinary();
  }
  if (xAdcUpdateSemaphore == NULL){ //confirm semaphore wasn't already made
    xAdcUpdateSemaphore = xSemaphoreCreateBinary();
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

  xTaskCreate(
    TaskReadAdcBrightness,
    (const portCHAR *)"Read ambient brightness",
    128, //stack size
    NULL,
    3, //priority
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

/***** System Control Functions *****/

void setBarColor(uint8_t bar_addr, uint8_t red, uint8_t green, uint8_t blue, uint8_t range_type){
  //pend semaphore
  uint8_t buffer[] = {0xAA, 0x00, 0x00, 0x00, 0x00, 0xBB};

  //package argument details into buffer
  //if leds are common cathode pass color values as-is
  if (range_type == NORMAL_RANGE){
    buffer[1] = bar_addr;
    buffer[2] = red * light_dimness;
    buffer[3]= green * light_dimness;
    buffer[4] = blue * light_dimness;
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

/**** Interrupt Service Routines *****/

//function to trigger animation disabling and 5 second timeout
//on receiver interrupt
void rcvrISR(void){
  //post led_disable_semaphore
  xSemaphoreGiveFromISR(xLedDisableSemaphore, NULL);
} //end of rcvrISR()

void adcISR(void){
  //post adc_update semaphore
  xSemaphoreGiveFromISR(xAdcUpdateSemaphore, NULL);
}

//interrupt service routine for timer 2
ISR (TIMER2_OVF_vect)
{
  timer_counter++;

  if (timer_counter == 30){
    timeout_counter++;
    timer_counter = 0;
  }

  if (timeout_counter == COOLDOWN_PERIOD){
    digitalWrite(debug_led, LOW);
    TCCR2B = (0 << CS22) | (0 << CS21) | (0 << CS20); //disable timer
    TCNT2 = 0x00; //clear counter
    timer_counter = 0;
    timeout_counter = 0;
    timer_done = 1; //signal that COOLDOWN_PERIOD seconds have passed
    //Serial.println(2);
  }

}

/***** RTOS Tasks *****/

//task to toggle through and execute animations
void TaskAnimate(void *pvParameters __attribute__((unused)) ){
  (void) pvParameters;

  while(1)
  {    
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
void TaskAnimationDisable(void *pvParameters __attribute__((unused)) )
{
  (void) pvParameters;
  
  while(1)
  {
    //pend on led_disable_semaphore
    if (xSemaphoreTake(xLedDisableSemaphore, portMAX_DELAY) == pdTRUE)
    {
      //loop through led bars and send message to disable animation
      disableLedBars(LED_BAR_COUNT, led_bar);
      
      //delay COOLDOWN_PERIOD seconds
      TCCR2B = 0 << CS20; //timer clock => disable for configuration
      TIFR2 = 1 << TOV2; //clear the overflow flag
      TIMSK2 = 1 << TOIE2; //enable timer interrupts
      TCNT2 = 0x00; //clear timer0 counter
      TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); //start timer with 1024 prescaler

      //loop until timer complete
      digitalWrite(debug_led, HIGH);
      vTaskSuspendAll();
      while (timer_done != 1){
        //Serial.println(1);
      }
      xTaskResumeAll();
      timer_done = 0;
      //Serial.println(3);
    }
  }
}

/**
 * Perform an analogRead() and return a scale by which
 * LED brightness must be reduced.
 */ 
void TaskReadAdcBrightness(void* pvParameters __attribute__((unused)) ){
  (void) pvParameters;

  while(1) //a task shall never exit or return
  {
    //pend AdcUpdateSemaphore
    if (xSemaphoreTake(xAdcUpdateSemaphore, portMAX_DELAY) == pdTRUE)
    {
      //record ambient_brightness
      uint16_t reading;
      reading = analogRead(A0);

      //calculate the dimming scale
      if (reading < 205){ 
        //100% brightness
        light_dimness = 1.0;
      }
      else if ((reading > 205) && (reading <= 410)){
        //80% brightness
        light_dimness = 0.8;
      }
      else if ((reading > 410) && (reading <= 615)){
        //60% brightness
        light_dimness = 0.5;
      }
      else if ((reading > 615) && (reading <= 820)){
        //40% brightness
        light_dimness = 0.4;
      }
      else{
        //20% brightness
        light_dimness = 0.2;
      }
    }
  }
}
