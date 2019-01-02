  //setup some global vars
  char id = '3';
  int state = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(8, OUTPUT);
  digitalWrite(8,LOW);
  
  //begin serial device
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  //if serial device is available, read a byte from it
  if (Serial.available() > 0){
    //read data from buffer
    char letter = Serial.read();
    
    //if the data id's us, respond
    if (letter == id){

      //and if state is 0 (off)
      if (state == 0){
      //set LED high
      digitalWrite(8, HIGH);
      state = 1;  
      }

      //else state must be 1 (led on)
      else{
      //set LED low
      digitalWrite(8, LOW);
      state = 0;
      }
      
    }
  }
  
} //end of loop
