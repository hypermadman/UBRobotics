#include <Servo.h>
#include <Car.h> //Include the library

Car car(10, 11, 2, 3, 6, 7, 13, 255, 8, 4); //Make a new instance of class 'car' called 'Car' with the following pins
Servo leftarm, rightarm;
int leftarmPin = 9;
int rightarmPin = 13;
const int pingPin = 13;
float M_PI = 3.14159265359;
int startPin = 12; //STARTING PIN

//SERVO SHIT


Servo myservo;  // create servo object to control a servo

int pos = 0;    // variable to store the servo position

/*
Refer to Car.cpp in the library for the meaning of the numbers

This code is the bare minimum required to get the chassis moving, and a bit more. Use the code as reference to make your own commands.
*/

void setup() {
Serial.begin(9600);
leftarm.attach(leftarmPin);
rightarm.attach(rightarmPin);
attachInterrupt(car.interruptPin[0], onPulse, CHANGE); //Attach interrupt pins. Even though we have set up interrupts for both tracks, simultaneously running two interrupts causes problems (workaround needed)

leftopen();
rightopen();
delay(100);
fishthing();
leftclose();
rightclose();

//WAIT FOR SWITCH
while(digitalRead(startPin) == LOW){
  delay(1);
}

//START EUROBOT green

car.move(1200,250); //PUSH BLOCK
delay(5000);
car.move(200,-250); //BACK OFF
car.gyroturn(M_PI/2); // TURN TO 1
leftopen(); //DEPLOY
rightopen();
car.move(750, 250); //Collect 1+2
leftclose();
car.gyroturn(M_PI/2);
leftopen();
car.move(650,250); //Collect 3
leftclose();
car.gyroturn(M_PI/2);
leftopen();
car.move(900,250); //Collect 4 back to start

//Close arms for fish

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
 // leftarm.write(175);
}
void leftopen(){
//  leftarm.write(55);
}
void rightclose(){
 // rightarm.write(55);
}
void rightopen(){
 // rightarm.write(175);
}
void fishthing(){
  delay(1000);
leftarm.detach();
rightarm.detach();
delay(100);
myservo.attach(5);
myservo.write(170);
delay(1000);
myservo.detach();

leftarm.attach(leftarmPin);
rightarm.attach(rightarmPin);
}


