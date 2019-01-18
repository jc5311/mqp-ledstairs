// rainbow func

// Define Variables
i = 0;
r = 255;
g = 0;
b = 0;
case = 1;
t = timer;
// dir is direction, 1 up and 0 down
dir = 1;

//start timer

// 5 minute while loop
while (t < 300000) {

// Switching between color incrementing
  switch(case) {
    case 1:
      setBarColor(led_bar[i], r, g, b, NORMAL_RANGE);
      g = g + 30;
      if (g >= 255) {
        g = 255;
        case = 2;
       }
      break;
      
    case 2:
      setBarColor(led_bar[i], r, g, b, NORMAL_RANGE);
      r = r - 30;
      if (r <= 0) {
        r = 0;
        case = 3;
       }
      break;
      
    case 3:
      setBarColor(led_bar[i], r, g, b, NORMAL_RANGE);
      b = b + 30;
      if (b >= 255) {
        b = 255;
        case = 4;
       }
      break;
      
    case 4:
      setBarColor(led_bar[i], r, g, b, NORMAL_RANGE);
      g = g - 30;
      if (g <= 0) {
        g = 0;
        case = 5;
       }
      break;    
      
    case 5:
      setBarColor(led_bar[i], r, g, b, NORMAL_RANGE);
      r = r + 30;
      if (r >= 255) {
        r = 255;
        case = 6;
       }
      break;
      
    case 6:
      setBarColor(led_bar[i], r, g, b, NORMAL_RANGE);
      b = b - 30;
      if (b <= 0) {
        b = 0;
        case = 1;
       }
      break; 
  }
  
// Tells direction of animation
  if (dir == 1){
    if (i == LED_BAR_COUNT) {
      dir = 0;
      }
    i = i + 1;
    }
  else {
    if (i == 1) {
      dir = 1;
      }
    i = i - 1;
    }
    
// Starts turning off Light bars once three are on at a time
  if (t > 2) {
    setBarColor(led_bar[i-2], 0, 0, 0, NORMAL_RANGE);
    }
    
// Delays one second
  delay(1000);
}
