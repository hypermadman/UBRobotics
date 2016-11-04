#include <Servo.h>
#include <MecanumRover.h>
#include <SPI.h>
#include <HCMAX7219.h>
#include <NewPing.h>
//Stepper stuff
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 1);

int leftArm = 40; //Pin numbers for seashell arms
int rightArm = 41;
Servo arm;
int bias = 90; //Arm will lower less with more bias
int pull = 47; //pull string

int rightAngle = 110;
int greenButton = 49; //Pin of green button for mode selection
int buzzerPin = 37; //Pin of buzzer
MecanumRover rover(22,5,7,9,
                   11,4,6,8,
                   18,19,2,3,
                   0,1,2,3,
                   60,10,49); //1-4 Direction Pins, 5-8 PWM Pins, 9-12 Interrupt Pins, 13-16 Current Read Pins, 15 Base Speed, 16 Correction Factor, 17 Buzzer Pin 

NewPing frontSensor(13, 12, 100);
NewPing rearSensor(31, 32, 100);

void setup(){
  // put your setup code here, to run once:
Serial.begin(9600);

AFMS.begin();  // create with the default frequency 1.6KHz
//AFMS.begin(1000);  // OR with a different frequency, say 1KHz

myMotor->setSpeed(100);  // 100 rpm   

//pinMode(greenButton, INPUT);
attachInterrupt(digitalPinToInterrupt(rover.interruptPin[0]), count0, CHANGE);
attachInterrupt(digitalPinToInterrupt(rover.interruptPin[1]), count1, CHANGE);
attachInterrupt(digitalPinToInterrupt(rover.interruptPin[2]), count2, CHANGE);
attachInterrupt(digitalPinToInterrupt(rover.interruptPin[3]), count3, CHANGE);
//rover.testHardware();

HCMAX7219 HCMAX7219(10);
Serial.println(digitalRead(greenButton));

myMotor->step(100, FORWARD, SINGLE); //600 lowers it 90
myMotor->step(100, BACKWARD, SINGLE);

leftClose();
rightClose();
rover.testHardware();

  if (digitalRead(greenButton) == HIGH){ //Check if GREEN BUTTON was pressed
    HCMAX7219.Clear();
    delay(1000);
    HCMAX7219.print7Seg("GREEN", 8); //Green mode
    delay(1000);
    HCMAX7219.Refresh();
    delay(1000);
    waiting();
  }
  else {
    HCMAX7219.Clear();
    delay(1000);
    HCMAX7219.print7Seg("PURPLE", 8); //Purple mode
    delay(1000);
    HCMAX7219.Refresh(); 
    delay(1000);
    waiting();
    eurobotPurple();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}

//Keep the following interrupt code simple and non-intensive for accurate operation

void count0(){
  rover.counter[0] ++;
  rover.encoderChangeTime[0] = millis();
}
void count1(){
  rover.counter[1] ++;
  rover.encoderChangeTime[1] = millis();
}
void count2(){
  rover.counter[2] ++;
  rover.encoderChangeTime[2] = millis();
}
void count3(){
  rover.counter[3] ++;
  rover.encoderChangeTime[3] = millis();
}

void leftClose(){
  arm.attach(leftArm);
    for (int pos = 180 - bias; pos >= 0; pos -= 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      arm.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  arm.detach();
}

void leftOpen(){
  arm.attach(leftArm);
    for (int pos = 0; pos <= 180 - bias; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      arm.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  arm.detach();
}

void rightClose(){
  arm.attach(rightArm);
    for (int pos = bias; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      arm.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  arm.detach();
}

void rightOpen(){
  arm.attach(rightArm);
    for (int pos = 180; pos >= 0 + bias; pos -= 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      arm.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  arm.detach();
}

void eurobotPurple(){
  rover.move(900,frontSensor, rearSensor);
  rover.move(-400,frontSensor, rearSensor);
  //Fish
  rover.turn(-rightAngle);
  rover.move(750,frontSensor, rearSensor);
  rover.turn(-rightAngle);
  rover.strafe(-300);
  //Lower the arm
  myMotor->step(600, FORWARD, SINGLE); 
  //rover.move(-400,frontSensor,rearSensor);
  myMotor->step(600, BACKWARD, SINGLE); 
  
  /* 
  rover.turn(-rightAngle);
  leftOpen();
  rightOpen(); //Open arms
  rover.move(500,frontSensor,rearSensor); //To 1,2
  rover.turn(-rightAngle);
  rover.move(500,frontSensor,rearSensor); //To 3
  rover.turn(-rightAngle);
  rover.move(1000,frontSensor,rearSensor); //To 4 then home
  */
}

void waiting(){

  pinMode(pull, INPUT);
  delay(1000);
  
  HCMAX7219 HCMAX7219(10);
  delay(1000);
  HCMAX7219.Clear();
  delay(1000);
  HCMAX7219.print7Seg("STANDBY", 8); //Purple mode
  delay(1000);
  HCMAX7219.Refresh(); 
  
  while (digitalRead(pull) == LOW){
    delay(1);
  }
}

