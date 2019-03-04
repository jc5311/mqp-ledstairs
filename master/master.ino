/**
 * Author(s):    Juan Caraballo, Shannon McCormack
 * Created:  3/4/19
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
#define UP 1
#define DOWN 0

//globals
uint8_t debug_led = 13;
uint8_t debug_toggle = 0;
uint8_t rcvr_interrupt_pin = 2; //only p2 and p3 can be used for interrupt on nano
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
volatile uint16_t anim_clock = 0;
volatile uint16_t start_anim_clock = FALSE;
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
  //0xAA signals start of lighting packet
  //0xAC signals ack packet
  //0xBB signals end of any packet
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

//interrupt service routine for timer 2
ISR (TIMER2_OVF_vect)
{
  timer_counter++;

  //if a second has passed
  if (timer_counter == 30)
  {
    //used for animations if they need it
    if (start_anim_clock == TRUE)
    {
      anim_clock++;
    }

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
    delay(100);
    setBarColor(led_bar[1], 0, 255, 0, NORMAL_RANGE);
    delay(100);
    setBarColor(led_bar[2], 0, 0, 255, NORMAL_RANGE);
    delay(100);
    setBarColor(led_bar[0], 0, 0, 0, NORMAL_RANGE);
    delay(100);
    setBarColor(led_bar[1], 0, 0, 0, NORMAL_RANGE);
    delay(100);
    setBarColor(led_bar[2], 0, 0, 0, NORMAL_RANGE);
    delay(100);
    //flip
    setBarColor(led_bar[2], 0, 0, 255, NORMAL_RANGE);
    delay(100);
    setBarColor(led_bar[1], 0, 255, 0, NORMAL_RANGE);
    delay(100);
    setBarColor(led_bar[0], 232, 12, 122, NORMAL_RANGE);
    delay(100);
    setBarColor(led_bar[2], 0, 0, 0, NORMAL_RANGE);
    delay(100);
    setBarColor(led_bar[1], 0, 0, 0, NORMAL_RANGE);
    delay(100);
    setBarColor(led_bar[0], 0, 0, 0, NORMAL_RANGE);
    delay(100);
    rain_travel();
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

void rainbowAnim(void)
{
  // rainbow func

  // Define Variables
  uint8_t i = 0;
  int16_t r = 255;
  int16_t g = 0;
  int16_t b = 0;
  uint8_t case_l = 1;
  //uint16_t t = timer;
  // dir is direction, 1 up and 0 down
  uint8_t dir = 1;
  uint8_t at_end = FALSE; //if at top or bottom

  //start timer
  start_anim_clock = TRUE;
  // 5 minute while loop
  while (anim_clock < 300000) 
  {

    // Switching between color incrementing
    switch(case_l) {
      case 1: //add green to red
        setBarColor(led_bar[i], r, g, b, NORMAL_RANGE);
        g = g + 30;
        if (g >= 255) {
          g = 255;
          case_l = 2;
        }
        break;
        
      case 2: //remove red from green
        setBarColor(led_bar[i], r, g, b, NORMAL_RANGE);
        r = r - 30;
        if (r <= 0) {
          r = 0;
          case_l = 3;
        }
        break;

      case 3: //add blue to green
        setBarColor(led_bar[i], r, g, b, NORMAL_RANGE);
        b = b + 30;
        if (b >= 255) {
          b = 255;
          case_l = 4;
        }
        break;
        
      case 4: //take out green from blue
        setBarColor(led_bar[i], r, g, b, NORMAL_RANGE);
        g = g - 30;
        if (g <= 0) {
          g = 0;
          case_l = 5;
        }
        break;   
        
      case 5: //add red to blue
        setBarColor(led_bar[i], r, g, b, NORMAL_RANGE);
        r = r + 30;
        if (r >= 255) {
          r = 255;
          case_l = 6;
        }
        break;
        
      case 6: //take out blue from red
        setBarColor(led_bar[i], r, g, b, NORMAL_RANGE);
        b = b - 30;
        if (b <= 0) {
          b = 0;
          case_l = 1;
        }
        break;
    }
      
    // Starts turning off Light bars once three are on at a time
    if (at_end == 1){
      at_end = 0;
      if (dir == UP)
      {
        i++;
      }
      else
      {
        i--;
      }
      
    }
    else if (i == LED_BAR_COUNT-1){
      setBarColor(led_bar[i-2], 0, 0, 0, NORMAL_RANGE);
      at_end = 1;
      dir = DOWN;
      i--;
    }
    else if (i == 0){
      setBarColor(led_bar[i+2], 0, 0, 0, NORMAL_RANGE);
      at_end = 1;
      dir = UP;
      i++;
    }
    else if (dir == 0){
      setBarColor(led_bar[i+2], 0, 0, 0, NORMAL_RANGE);
      i--;
    }
    else if (dir == 1){
      setBarColor(led_bar[i-2], 0, 0, 0, NORMAL_RANGE);
      i++;
    }
    // Delays one second
    delay(500);
  }
  //use this to time the animation for now
  start_anim_clock = FALSE;
  anim_clock = 0;
}



void colorMix (void) 
{

  // colorMix func

  // Define Variables

  // Counter
  int16_t i;
  
  // Define two colors by randoming selecting rgb values
  int16_t r1;
  int16_t g1;
  int16_t b1;
  int16_t r2;
  int16_t g2;
  int16_t b2;

  // Mixed values
  int16_t r3;
  int16_t b3;
  int16_t g3;
    
  // Percent of r3, b3. and g3 to fade
  int16_t r_fade_perc;
  int16_t b_fade_perc;
  int16_t g_fade_perc;

  // Cases for what to light up
  uint8_t case_m = 1;
  
  //uint16_t t = timer;

  //start timer
  start_anim_clock = TRUE;
  
  // 5 minute while loop
  while (anim_clock < 300000) {

    switch(case_m) {
      // Initialize values
      case 1:
        // Set counter to zero
        i = 0;
        
        // Set initial random rgb values
        r1 = rand() % 127;
        g1 = rand() % 127;
        b1 = rand() % 127;
        r2 = rand() % 127;
        g2 = rand() % 127;
        b2 = rand() % 127;
        
        // Add the two r values for the mixed value of r
        r3 = r1 + r2;
          
        // Add the two b values for the mixed value of b
        b3 = b1 + b2;
         
        // Add the two g values for the mixed value of g
        g3 = g1 + g2;
        
        // Percent of r3, b3. and g3 to fade
        r_fade_perc = r3 / (LED_BAR_COUNT/2);
        b_fade_perc = b3 / (LED_BAR_COUNT/2);
        g_fade_perc = g3 / (LED_BAR_COUNT/2);
  
        case_m = 2;
  
        break;
      
      // Before reaching the middle, turning LEDs on from top and bottom using colors 1 and 2
      case 2:
        setBarColor(led_bar[i], r1, g1, b1, NORMAL_RANGE);
        setBarColor(led_bar[LED_BAR_COUNT - i - 1], r2, g2, b2, NORMAL_RANGE);
        delay(1000);
        
        // Turn off LEDs three away
        if (i >= 3) {
          setBarColor(led_bar[i-3], 0, 0, 0, NORMAL_RANGE);
          setBarColor(led_bar[LED_BAR_COUNT - i + 2], 0, 0, 0, NORMAL_RANGE);
        } //end if (turn off LEDs)

        i++;

        // Check if met middle
        // If LED_BAR_COUNT is even
        if (i >= LED_BAR_COUNT - i - 1) {
          // If LED_BAR_COUNT is even
          if (LED_BAR_COUNT % 2 == 0) {
            case_m = 3;
          }
          // If LED_BAR_COUNT is odd
          else {
            case_m = 4;
          }
        }

        break;
        
      // Reached middle (even), setting LEDs to new color and turning off past LEDs
      case 3:
        // Set middle value(s) to one third of the mixed color
        setBarColor(led_bar[i], r3/3, g3/3, b3/3, NORMAL_RANGE);
        setBarColor(led_bar[LED_BAR_COUNT - i - 1], r3/3, g3/3, b3/3, NORMAL_RANGE);
        delay(1000);
        
        // Turn off LEDs 2 away from middle(s) and set middle value(s) to two thirds of the mixed color
        setBarColor(led_bar[i-2], 0, 0, 0, NORMAL_RANGE);
        setBarColor(led_bar[LED_BAR_COUNT - i + 1], 0, 0, 0, NORMAL_RANGE);
        setBarColor(led_bar[i], 2*r3/3, 2*g3/3, 2*b3/3, NORMAL_RANGE);
        setBarColor(led_bar[LED_BAR_COUNT - i - 1], 2*r3/3, 2*g3/3, 2*b3/3, NORMAL_RANGE);
        delay(1000);
        
        // Turn off LEDs 1 away from middle(s) and set middle value(s) to two thirds of the mixed color
        setBarColor(led_bar[i-1], 0, 0, 0, NORMAL_RANGE);
        setBarColor(led_bar[LED_BAR_COUNT - i], 0, 0, 0, NORMAL_RANGE);
        setBarColor(led_bar[i], r3, g3, b3, NORMAL_RANGE);
        setBarColor(led_bar[LED_BAR_COUNT - i - 1], r3, g3, b3, NORMAL_RANGE);
        delay(1000);

        case_m = 5;
        i++;

        break;

      // Reached middle (odd), setting LEDs to new color and turning off past LEDs
      case 4:
        // Set middle value(s) to one third of the mixed color
        setBarColor(led_bar[i], r3/3, g3/3, b3/3, NORMAL_RANGE);
        delay(1000);
        
        // Turn off LEDs 2 away from middle(s) and set middle value(s) to two thirds of the mixed color
        setBarColor(led_bar[i+2], 0, 0, 0, NORMAL_RANGE);
        setBarColor(led_bar[i-2], 0, 0, 0, NORMAL_RANGE);
        setBarColor(led_bar[i], 2*r3/3, 2*g3/3, 2*b3/3, NORMAL_RANGE);
        delay(1000);
        
        // Turn off LEDs 1 away from middle(s) and set middle value(s) to two thirds of the mixed color
        setBarColor(led_bar[i+1], 0, 0, 0, NORMAL_RANGE);
        setBarColor(led_bar[i-1], 0, 0, 0, NORMAL_RANGE);
        setBarColor(led_bar[i], r3, g3, b3, NORMAL_RANGE);
        delay(1000);

        case_m = 5;
        i++;

        break;

      // Past middle, setting new color to LEDs
      case 5:
        // Set new color to LEDs
        setBarColor(led_bar[i], r3, g3, b3, NORMAL_RANGE);
        setBarColor(led_bar[LED_BAR_COUNT - i - 1], r3, g3, b3, NORMAL_RANGE);
        delay(1000);
        
        // Turn off LEDs 3 away
        if (i >= LED_BAR_COUNT - i + 5) {
          setBarColor(led_bar[i-3], 0, 0, 0, NORMAL_RANGE);
          setBarColor(led_bar[LED_BAR_COUNT - i + 2], 0, 0, 0, NORMAL_RANGE);
        } // end if (turn off LEDs)
        
        // Fade out color
        r3 = r3 - r_fade_perc;
        b3 = b3 - b_fade_perc;
        g3 = g3 - g_fade_perc;

        // Restart when all LEDs have been truned off
        if (i == LED_BAR_COUNT + 2) {
          case_m = 1;
        }
        else {
          i++;
        }

        break;
    } // end case
    
  } //end while
  
  //use this to time the animation for now
  start_anim_clock = FALSE;
  anim_clock = 0;
} //end function


void travel(int r, int b, int g, int num_leds)
{
  for (int i = 0; i <= num_leds; i++){
    if (i == 0){
      setBarColor(led_bar[i], r, g, b, NORMAL_RANGE);
      delay(250);
    }
    else {
      setBarColor(led_bar[i], r, g, b, NORMAL_RANGE);
      delay(250);
      setBarColor(led_bar[i-1], 0, 0, 0, NORMAL_RANGE);
    }
  }
}

void rain_travel(void)
{
  // traveling rainbow func

  // Define Variables
  int16_t i = LED_BAR_COUNT - 1;
  int16_t r = 255;
  int16_t g = 0;
  int16_t b = 0;
  uint8_t case_t = 1;
  //incrementing color
  int16_t color_inc = 1530/LED_BAR_COUNT;

  //start timer
  start_anim_clock = TRUE;
  // 5 minute while loop
  while (anim_clock < 300000) 
  {
    // Initial Color Set
    switch(case_t) {
      case 1: //add green to red
        travel(r, b, g, i);
        g = g + color_inc;
        if (g >= 255) {
          g = 255;
          case_t = 2;
        }
        break;
        
      case 2: //remove red from green
        travel(r, b, g, i);
        r = r - color_inc;
        if (r <= 0) {
          r = 0;
          case_t = 3;
        }
        break;

      case 3: //add blue to green
        travel(r, b, g, i);
        b = b + color_inc;
        if (b >= 255) {
          b = 255;
          case_t = 4;
        }
        break;
        
      case 4: //take out green from blue
        travel(r, b, g, i);
        g = g - color_inc;
        if (g <= 0) {
          g = 0;
          case_t = 5;
        }
        break;   
        
      case 5: //add red to blue
        travel(r, b, g, i);
        r = r + color_inc;
        if (r >= 255) {
          r = 255;
          case_t = 6;
        }
        break;
        
      case 6: //take out blue from red
        travel(r, b, g, i);
        b = b - color_inc;
        if (b <= 0) {
          b = 0;
          case_t = 1;
        }
        break;
      case 7: //Turn off LEDs
        for (int j = LED_BAR_COUNT - 1; j >= 0; j--) {
          setBarColor(led_bar[j], 0, 0, 0, NORMAL_RANGE);
          delay(250);
        }
        case_t = 1;
        break;
    }
      
    // Starts turning off Light bars once three are on at a time
    if (i == 0) {
      case_t = 7;
      i = LED_BAR_COUNT;
      r = 255;
      g = 0;
      b = 0;
    }
    else {
      i--;
    }
  }
  //use this to time the animation for now
  start_anim_clock = FALSE;
  anim_clock = 0;
}

void ak_stairs(void)
{
  // message func

  // Define Variables
  // Message incrementer
  int16_t j = 0;
  int16_t temp;
  //Switch case
  uint8_t case_mess = 1;

  //start timer
  start_anim_clock = TRUE;
  // 5 minute while loop
  while (anim_clock < 300000) 
  {
    // Message
    int message[63] = {1,0,0,0,0,0,1,1,1,0,1,0,0,1,0,0,0,0,0,1,0,1,1,0,0,1,0,1,0,0,1,0,1,1,1,1,0,0,0,0,1,1,1,0,0,1,0,1,1,0,1,0,0,1,1,1,1,1,0,0,1,1,1};
    while (j <= 62) 
    {
      //Write A in Red
      for (int i = 0; i <= LED_BAR_COUNT - 1; i++) {
        switch (case_mess) {
          // ASCII Char in Red
          case 1:
            if (temp){
              setBarColor(led_bar[i], 255, 0, 0, NORMAL_RANGE);
            }
            else {
              setBarColor(led_bar[i], 0, 0, 0, NORMAL_RANGE);
            }
            break;
          // ASCII Char in Yellow
          case 2:
            if (temp){
              setBarColor(led_bar[i], 255, 255, 0, NORMAL_RANGE);
            } 
            else {
              setBarColor(led_bar[i], 0, 0, 0, NORMAL_RANGE);
            }
            break;
          // ASCII Char in Green
          case 3:
            if (temp){
              setBarColor(led_bar[i], 0, 255, 0, NORMAL_RANGE);
            } 
            else {
              setBarColor(led_bar[i], 0, 0, 0, NORMAL_RANGE);
            }
            break;
          // ASCII Char in Cyan
          case 4:
            if (temp){
              setBarColor(led_bar[i], 0, 255, 255, NORMAL_RANGE);
            } 
            else {
              setBarColor(led_bar[i], 0, 0, 0, NORMAL_RANGE);
            }
            break;
          // ASCII Char in Blue
          case 5:
            if (temp){
              setBarColor(led_bar[i], 0, 0, 255, NORMAL_RANGE);
            } 
            else {
              setBarColor(led_bar[i], 0, 0, 0, NORMAL_RANGE);
            }
            break;
          // ASCII Char in Violet
          case 6:
            if (temp){
              setBarColor(led_bar[i], 255, 0, 255, NORMAL_RANGE);
            } 
            else {
              setBarColor(led_bar[i], 0, 0, 0, NORMAL_RANGE);
            }
            break;
        }
        j++;
        temp = message[j];
        if ((j <= 6) || (j > 41 && j <= 48)){
          case_mess = 1;
        }
        else if ((j > 6 && j <= 13) || (j > 48 && j <= 55)){
          case_mess = 2;
        }
        else if ((j > 13 && j <= 20) || (j > 55 && j <= 62)){
          case_mess = 3;
        }
        else if (j > 20 && j <= 27){
          case_mess = 4;
        }
        else if (j > 27 && j <= 34){
          case_mess = 5;
        }
        else{
          case_mess = 6;
        }
        
      }
      delay(1000);
      for (int i = 0; i <= LED_BAR_COUNT - 1; i++){
        setBarColor(led_bar[i], 0, 0, 0, NORMAL_RANGE);
      }
      j = j - LED_BAR_COUNT + 1;
      temp = message[j];
      if ((j <= 6) || (j > 41 && j <= 48)){
        case_mess = 1;
      }
      else if ((j > 6 && j <= 13) || (j > 48 && j <= 55)){
        case_mess = 2;
      }
      else if ((j > 13 && j <= 20) || (j > 55 && j <= 62)){
        case_mess = 3;
      }
      else if (j > 20 && j <= 27){
        case_mess = 4;
      }
      else if (j > 27 && j <= 34){
        case_mess = 5;
      }
      else{
        case_mess = 6;
      }
    }
    j = 0;
    temp = message[j];
    if ((j <= 6) || (j > 41 && j <= 48)){
      case_mess = 1;
    }
    else if ((j > 6 && j <= 13) || (j > 48 && j <= 55)){
      case_mess = 2;
    }
    else if ((j > 13 && j <= 20) || (j > 55 && j <= 62)){
      case_mess = 3;
    }
    else if (j > 20 && j <= 27){
      case_mess = 4;
    }
    else if (j > 27 && j <= 34){
      case_mess = 5;
    }
    else{
      case_mess = 6;
    }
  }
  //use this to time the animation for now
  start_anim_clock = FALSE;
  anim_clock = 0;
}
