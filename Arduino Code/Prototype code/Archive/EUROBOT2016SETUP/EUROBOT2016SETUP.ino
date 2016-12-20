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

leftclose(); //RETRACT ARMS
rightclose();

//WAIT FOR SWITCH
while(digitalRead(startPin) == LOW){
  delay(1);
}

//START EUROBOT green

car.move(1200,250);
delay(5000);
car.move(500,-250);
car.gyroturn(-M_PI/2);
leftopen();
rightopen();
car.move(600, 250); //Collect 1+2
rightclose();
car.gyroturn(-M_PI/2);
rightopen();
car.move(500,250); //Collect 3
rightclose();
car.gyroturn(-M_PI/2);
rightopen();
car.move(600,250); //Collect 4 back to start
car.turn(200); //Turn a bit to push them back into the beach towel

fishthing();
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
  //leftarm.write(175);
}
void leftopen(){
  //leftarm.write(55);
}
void rightclose(){
 // rightarm.write(55);
}
void rightopen(){
 // rightarm.write(175);
}
void fishthing(){
leftarm.detach();
rightarm.detach();
  
  myservo.detach();
  myservo.attach(0);  // attaches the servo on pin 0 to the servo object
  for(pos = 100; pos>=25; pos-=1)     // goes from 180 degrees to 0 degrees GODOWN
  {
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  myservo.detach();
  myservo.attach(5);
    for(pos = 5; pos <= 160; pos += 1) // goes from 0 degrees to 180 degrees ROTATE
  {                                  // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  myservo.detach();
  myservo.attach(0);
  for(pos = 25; pos<=50; pos+=1)     // goes from 180 degrees to 0 degrees UP A BIT
  {
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  delay(1000);
  for(pos = 50; pos>=25; pos-=1)     // goes from 180 degrees to 0 degrees DOWN A BIT
  {
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  delay(1000);
  myservo.detach();
  myservo.attach(5);
    for(pos = 160; pos >= 5; pos -= 1) // goes from 0 degrees to 180 degrees ROTATE
  {                                  // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  myservo.detach();
  myservo.attach(0);  // attaches the servo on pin 0 to the servo object
  for(pos = 25; pos<=100; pos+=1)     // goes from 180 degrees to 0 degrees GOUP
  {
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
leftarm.attach(leftarmPin);
rightarm.attach(rightarmPin);
}


