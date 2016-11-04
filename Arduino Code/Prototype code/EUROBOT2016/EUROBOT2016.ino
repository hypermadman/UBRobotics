#include <Servo.h>
#include <Car.h> //Include the library

Car car(10, 11, 2, 3, 6, 7, 13, 255, 8, 4); //Make a new instance of class 'car' called 'Car' with the following pins
Servo leftarm, rightarm;
int leftarmPin = 9;
int rightarmPin = 13;
const int pingPin = 13;
float M_PI = 3.14159265359;
int startPin = 12; //STARTING PIN

/*
Refer to Car.cpp in the library for the meaning of the numbers

This code is the bare minimum required to get the chassis moving, and a bit more. Use the code as reference to make your own commands.
*/

void setup() {
Serial.begin(9600);
leftarm.attach(leftarmPin);
rightarm.attach(rightarmPin);
attachInterrupt(car.interruptPin[0], onPulse, CHANGE); //Attach interrupt pins. Even though we have set up interrupts for both tracks, simultaneously running two interrupts causes problems (workaround needed)

leftclose(); //RETRACT ARMS
rightclose();

//WAIT FOR SWITCH
while(digitalRead(startPin) == LOW){
  delay(1);
}

//START EUROBOT yellow
car.move(1200,250);
car.move(700,-250);
car.gyroturn(M_PI/2);
leftopen();
rightopen();
car.move(600, 250); //Collect 1+2
leftclose();
car.gyroturn(M_PI/2);
leftopen();
car.move(350,250); //Collect 3
leftclose();
car.gyroturn(M_PI/2);
leftopen();
car.move(600,250); //Collect 4 back to start
car.turn(-200); //Turn a bit to push them back into the beach towel
}

void loop(){
}

void onPulse() {
  
  if ((car.state[0] == false) && (digitalRead(car.interruptPin[0]) == HIGH)){
    car.counter[0] ++;
    car.state[0] = true;
  }
  else if ((car.state[0] == true) && (digitalRead(car.interruptPin[0]) == LOW)){
    car.counter[0] ++;
    car.state[0] = false;
  }
}

void leftclose(){
  leftarm.write(175);
}
void leftopen(){
  leftarm.write(55);
}
void rightclose(){
  rightarm.write(55);
}
void rightopen(){
  rightarm.write(175);
}

