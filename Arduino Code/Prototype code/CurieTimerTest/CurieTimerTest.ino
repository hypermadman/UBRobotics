#include <CurieTimerOne.h>

/*
  CurieTimer has a limit of 1000000 microseconds = 1 second
  Consider incrementing a counter every second to get longer duration timers

  Will implement into maybe a neater looking library of its own
 */
int count = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
  pinMode(4, OUTPUT);
  CurieTimerOne.start(1000000, &Blink);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);              // wait for a second
}

void Blink(){
  if (count < 10){ //Less than 10 seconds
    count ++; //Add a second to the counter
    CurieTimerOne.start(1000000, &Blink); //Restart the timer, for some reason CurieTimerOne.restart() doesnt do the trick, prehaps because the timer has ended?
  }
  else {
    digitalWrite(4,HIGH); //After count reaches 10, it's been 10 seconds
    count = 0; //Reset it
  }
}

