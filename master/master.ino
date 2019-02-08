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
#define TRUE 1
#define FALSE 0

//globals
uint8_t debug_led = 13;
uint8_t debug_toggle = 0;
uint8_t rcvr_interrupt_pin = 2; //only p2 and p3 can be used for interrupt on nano
uint8_t adc_interrupt_pin = 3; //only p2 and p3 can be used for interrupt on nano
uint8_t led_bar[LED_BAR_COUNT]; //array to hold led bar addresses
uint8_t toggler = 0;
float light_dimness = 1;
//variables used in interrupts must ALWAYS be declared as volatile
volatile uint8_t cooldown_done = 0; //signal to resume normal animation
volatile uint8_t timer_counter = 0; // counter used for 1 second timing
volatile uint8_t cooldown_counter = 0; //counter used during cooldown
volatile uint16_t time_since_last_rcvr_int = 0; //used for triggering low power mode
volatile uint16_t lowpower_counter = 0; // used for low power mode timing
volatile uint16_t adc_counter = 3599; //trigger brightness update; trigger on startup
volatile uint8_t cooldown_already_running = FALSE;
volatile uint8_t animation_disable = FALSE; //signal to disable animations
volatile uint8_t lowpower_sleep = FALSE; //use for actual sleeping during low power mode
volatile uint8_t lowpower_mode_active = FALSE;

//prototypes
void setBarColor(uint8_t bar_addr, uint8_t red, 
                 uint8_t green, uint8_t blue, uint8_t range_type);
void disableLedBars(uint8_t led_bar_count, uint8_t led_bar[]);
void rcvrISR(void);
void cooldownTimer(void);
//rtos tasks
void TaskAnimate(void* pvParameters);
void Cooldown(void* pvParameters);
void TaskReadAdcBrightness(void* pvParameters);

//rtos semaphores
SemaphoreHandle_t xLedDisableSemaphore;
SemaphoreHandle_t xAdcUpdateSemaphore;
//rtos task handles
TaskHandle_t xAnimateHandle;


void setup()
{
  //debug led setup
  pinMode(debug_led, OUTPUT);
  digitalWrite(debug_led, LOW);

  /********** Initialize and configure interrupts **********/
  pinMode(rcvr_interrupt_pin, INPUT_PULLUP);
  //attach interrupt to the interrupt pin and configure trigger on falling edge
  attachInterrupt(digitalPinToInterrupt(rcvr_interrupt_pin), rcvrISR, FALLING);
  //do the same for the adc for demoing
  pinMode(adc_interrupt_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(adc_interrupt_pin), adcISR, FALLING);

  /********** Configure and start serial port **********/
  Serial.begin(BAUD_RATE);
  while(!Serial); //wait for serial port to connect

  /********** Initialize led bar addresses **********/
  led_bar[0] = 0xAB;
  led_bar[1] = 0xBC;
  led_bar[2] = 0xCD;

  /********** Configure Semaphores **********/
  if (xLedDisableSemaphore == NULL)
  { 
    //confirm semaphore wasn't already made
    xLedDisableSemaphore = xSemaphoreCreateBinary();
  }
  if (xAdcUpdateSemaphore == NULL)
  { 
    //confirm semaphore wasn't already made
    xAdcUpdateSemaphore = xSemaphoreCreateBinary();
  }

  /********** Configure and start system timer **********/
  TCCR2B = 0 << CS20; //timer clock => disable for configuration
  TIFR2 = 1 << TOV2; //clear the overflow flag
  TIMSK2 = 1 << TOIE2; //enable timer interrupts
  TCNT2 = 0x00; //clear timer0 counter
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); //start timer with 1024 prescaler

  /********** Configure RTOS Tasks **********/
  xTaskCreate(
    TaskAnimate,
    (const portCHAR *)"Toggle Animations",
    128, //stack size
    NULL,
    1, //priority
    &xAnimateHandle
  );

  xTaskCreate(
    TaskLedDisable,
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

void loop()
{
  //We are using an rtos therefore all work should be placed in tasks!
}

/********** System Control Functions **********/

void setBarColor(uint8_t bar_addr, uint8_t red, uint8_t green, uint8_t blue, uint8_t range_type){
  //pend semaphore
  uint8_t buffer[] = {0xAA, 0x00, 0x00, 0x00, 0x00, 0xBB};

  //package argument details into buffer
  //if leds are common cathode pass color values as-is
  if (range_type == NORMAL_RANGE)
  {
    buffer[1] = bar_addr;
    buffer[2] = red * light_dimness;
    buffer[3]= green * light_dimness;
    buffer[4] = blue * light_dimness;
  }
  //if leds are common anode reverse the color values
  else if (range_type == FLIPPED_RANGE)
  {
    buffer[1] = bar_addr;
    buffer[2] = EIGHTBITMAX - red;
    buffer[3]= EIGHTBITMAX - green;
    buffer[4] = EIGHTBITMAX - blue;
  }

  //Write buffer to serial
  Serial.write(buffer, sizeof(buffer));

  //post semaphore
} //end of setBarColor()

void disableLedBars(uint8_t led_bar_count, uint8_t led_bar[])
{
  //loop through the number of bars and send a message to turn off
  for (int i = 0; i < led_bar_count; i++){
    setBarColor(led_bar[i], 0, 0, 0, NORMAL_RANGE);
  }
} //end of disableLedBars()

/**** Interrupt Service Routines *****/

//function to trigger animation disabling and 5 second timeout
//on receiver interrupt
void rcvrISR(void){
  if (!cooldown_already_running)
  {
    //absolutely disable lowpower_mode
    lowpower_mode_active = FALSE;
    lowpower_counter = 0;
    time_since_last_rcvr_int = 0;
    //post led_disable_semaphore
    xSemaphoreGiveFromISR(xLedDisableSemaphore, NULL);
  }
  else //if timer is currently active reset its counter
  {
    time_since_last_rcvr_int = 0;
    cooldown_counter = 0;
    timer_counter = 0;
  }
  
} //end of rcvrISR()

void adcISR(void){
  //post adc_update semaphore
  xSemaphoreGiveFromISR(xAdcUpdateSemaphore, NULL);
}

//interrupt service routine for timer 2
ISR (TIMER2_OVF_vect)
{
  timer_counter++;

  //if a second has passed
  if (timer_counter == 30)
  {
    //reset the one second signal
    timer_counter = 0;

    //incremement and check counter for time since last receiver interrupt
    //to enable low power mode
    time_since_last_rcvr_int++;
    if (time_since_last_rcvr_int == 1800) //if 30m have passed
    {
      lowpower_mode_active = TRUE;
      lowpower_counter = 0;
      lowpower_sleep = FALSE;
    }
    
    //low power mode handler
    if (lowpower_mode_active == TRUE)
    {
      if (lowpower_sleep)
      {
        if (lowpower_counter == 480) //if 8 minutes have passed
        {
          lowpower_counter = 0;
          lowpower_sleep = FALSE; //disable animation waiting
          vTaskResume(xAnimateHandle); //resume animation task
        }
      }
      else //if we aren't waiting to resume animation
      {
        if (lowpower_counter == 120) //if two minutes of animation has passed
        {
          lowpower_counter = 0;
          lowpower_sleep = TRUE;
          xSemaphoreGiveFromISR(xLedDisableSemaphore, NULL);
          vTaskSuspend(xAnimateHandle); //suspend animation task
        }
      }
      lowpower_counter++;
    }


    //if animation_disable is set due to receiver interrupt,
    //handle its timing    
    if (animation_disable == TRUE)
    {
      //do all animation disable (trip sensor) stuff here
      cooldown_counter++;
      if (cooldown_counter == COOLDOWN_PERIOD){
        cooldown_counter = 0;
        cooldown_done = 1; //signal that COOLDOWN_PERIOD seconds have passed
      }
    }

    //handle adc updates every hour
    adc_counter++;
    if (adc_counter == 3600) //if an hour has passed
    {
      adc_counter = 0;
      xSemaphoreGiveFromISR(xAdcUpdateSemaphore, NULL);
    }
  }

}

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
void TaskLedDisable(void *pvParameters __attribute__((unused)) )
{
  (void) pvParameters;
  
  while(1)
  {
    //pend on led_disable_semaphore
    if (xSemaphoreTake(xLedDisableSemaphore, portMAX_DELAY) == pdTRUE)
    {
      if (lowpower_sleep)
      {
        disableLedBars(LED_BAR_COUNT, led_bar);
      }
      else
      {
        //signal that timeout is currently running, this fixes bug where
        //2 successive interrupts cause a 10 second cooldown instead of 5 after
        //the last
        cooldown_already_running = TRUE;
        
        //loop through led bars and send message to disable animation
        disableLedBars(LED_BAR_COUNT, led_bar);
        
        //set signal for timer to acknowledge animation disable
        animation_disable = TRUE;

        //loop until timer complete
        vTaskSuspend(xAnimateHandle); //suspend animation task
        while (cooldown_done != TRUE);
        vTaskResume(xAnimateHandle); //resume animation task
        cooldown_done = FALSE; //reset the cooldown_done signal
        cooldown_already_running = FALSE;
        animation_disable = FALSE;
      }
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
