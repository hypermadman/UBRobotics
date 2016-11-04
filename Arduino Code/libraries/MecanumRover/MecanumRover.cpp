#include "Arduino.h"
#include "MecanumRover.h"
#include <SPI.h>
#include <HCMAX7219.h>

int brakeTime = 20; //20 nominal //DEF 20
long startTime;
bool strafing = false; //Are we strafing diagonally EXPERIMENTAL
int distanceLimit = 40;
int lastEncoder[4]; //Previous encoder position 
HCMAX7219 HCMAX7219(10); // SETUP THE LED DISPLAY (Pin 10 = CS 51 = DIN 52 = CLK)

MecanumRover::MecanumRover(int dir0, int dir1, int dir2, int dir3, int pwm0, int pwm1, int pwm2, int pwm3, int int0, int int1, int int2, int int3, int cur0, int cur1, int cur2, int cur3, int base, float factor, int buzzerPin){
  buzzer = buzzerPin;

  directionPin[0] = dir0; //Initialise direction pins
  directionPin[1] = dir1;
  directionPin[2] = dir2;
  directionPin[3] = dir3;

  pwmPin[0] = pwm0; //Initialise PWM pins
  pwmPin[1] = pwm1; 
  pwmPin[2] = pwm2; 
  pwmPin[3] = pwm3;

  interruptPin[0] = int0; //Initialise interrupt pins
  interruptPin[1] = int1;
  interruptPin[2] = int2;
  interruptPin[3] = int3;

  currentPin[0] = cur0; //Initialise current Pins
  currentPin[1] = cur1; 
  currentPin[2] = cur2; 
  currentPin[3] = cur3; 

  baseSpeed = base; //Initialise base speed

  correctionFactor = factor; //Initialise correction factor

  for (int i = 0; i < 4; i ++){
    pinMode(directionPin[i], OUTPUT); //DirectionPin is a digital output pin
    pinMode(interruptPin[i], INPUT);  //InterruptPin is an input
    digitalWrite(interruptPin[i], HIGH); //A pullup resistor to protect Arduino as interrupt pin is 5v logic
  }
}

void MecanumRover::adjustSpeed(int ticks){ //Change the speed of the 4 wheels according to the average rotational displacement of all 4 wheels
  int averageCount = (counter[0] + counter[1] + counter [2] + counter[3]) / 4;  //Average rotational displacement of all 4 wheels
  
  /*
  Serial.print("Displacement: ");
  Serial.print(counter[0]);
  Serial.print(" ");
  Serial.print(counter[1]);
  Serial.print(" ");
  Serial.print(counter[2]);
  Serial.print(" ");
  Serial.print(counter[3]); 
  Serial.print(" Speed: ");
  */

  for (int i = 0; i < 4; i ++){
    if ((counter[i] > 0) &&(sensorStop == false)){ //To make sure all wheels start up at the same speed avoid correction until a certain displacement, and that the sensor is not detecting anything
      int correction = averageCount - counter[i]; //Difference of current wheel from average
      
        if (correction > speedDeltaCap){
          correction = speedDeltaCap; //To avoid wheel speeds to increase too much on correction there is a cap
        }
        else if (correction < -speedDeltaCap){
          correction = -speedDeltaCap;
        }

      int adjusted = baseSpeed + correction * correctionFactor; //Change PWM level according to difference
      //Serial.print(adjusted);
      //Serial.print(" ");
      //int travelRatio = (abs(ticks) - (averageCount+1)) / abs(ticks); //Percentage difference in travelled distance vs intended
      if (strafing == true){
        analogWrite(pwmPin[i], adjusted); //Write to PWM pin
      }
      else {
        analogWrite(pwmPin[i], adjusted); //Write to PWM pin 
      }
    }
    currentCurrent[i] = analogRead(currentPin[i]);
  }
  //Serial.println();
}

void MecanumRover::move(int ticks, NewPing frontSensor, NewPing rearSensor){ //Move forward/backward a number of ticks

  if (ticks > 0){ //Forwards
    digitalWrite(directionPin[0], LOW);
    digitalWrite(directionPin[1], LOW);
    digitalWrite(directionPin[2], HIGH);
    digitalWrite(directionPin[3], HIGH);

  }
  else {
    digitalWrite(directionPin[0], HIGH);
    digitalWrite(directionPin[1], HIGH);
    digitalWrite(directionPin[2], LOW);
    digitalWrite(directionPin[3], LOW);
  }

  startTime = millis();

  for (int i = 0; i < 4; i ++){
    counter[i] = 0; //Initialise encoder count
    analogWrite(pwmPin[i], baseSpeed);
  }

  while(counter[0] + counter[1] + counter[2] + counter[3] < abs(ticks) * 4){ //While total sum of encoders for all wheels is less than the desired distance x4
    //Check if there is anything at the front/rear
    if (ticks > 0){
      int distance = frontSensor.ping_cm();

      if ((distance <= distanceLimit) && (distance != 0)){ //When the sensor has nothing in front of it, it gives zero. There is an obstruction when distance < limit and not 0
        for (int i = 0; i < 4; i ++){ //For all 4 wheels
           analogWrite(pwmPin[i], 0); //Off
        }
        sensorStop = true;
        Serial.print("Stopped moving forwards at: ");
        Serial.println(millis());
        //tone(buzzer, 220, 1000);
      }
      else { //Sensor is not obstructing
        for (int i = 0; i < 4; i ++){ //For all 4 wheels
           analogWrite(pwmPin[i], baseSpeed); //On
        }
        sensorStop = false;
      }
    }
    else if (ticks < 0){
      int distance = rearSensor.ping_cm();

      if ((distance <= distanceLimit) && (distance != 0)){ //When the sensor has nothing in front of it, it gives zero. There is an obstruction when distance < limit and not 0
        for (int i = 0; i < 4; i ++){ //For all 4 wheels
           analogWrite(pwmPin[i], 0); //Off
        }
        sensorStop = true;
        Serial.print("Stopped moving backwards at: ");
        Serial.println(millis());
        //tone(buzzer, 220, 1000);
      }
      else { //Sensor is not obstructing
        for (int i = 0; i < 4; i ++){ //For all 4 wheels
           analogWrite(pwmPin[i], baseSpeed); //On
        }
        sensorStop = false;
      }
    }
    

    adjustSpeed(ticks);
   // checkIfStuck(abs(ticks)); //Check for stuck, if stuck then break out
  }  
  brake(); 
  for (int i = 0; i < 4; i ++){ //For all 4 wheels
    analogWrite(pwmPin[i], 0); //Off
  }

  printResults(ticks);
}

void MecanumRover::testHardware(){
    digitalWrite(directionPin[0], LOW);
    digitalWrite(directionPin[1], LOW);
    digitalWrite(directionPin[2], HIGH);
    digitalWrite(directionPin[3], HIGH);

  for (int i = 0; i < 4; i ++){
    counter[i] = 0; //Initialise encoder count
    analogWrite(pwmPin[i], 5); //slow speed
  }  
  delay(1000);
  Serial.println("HARDWARE TEST (ZERO = ERROR):");

  Serial.print("Encoder readings: ");
    for (int i = 0; i < 4; i ++){
      Serial.print(counter[i]);
      Serial.print(" ");
    }
  Serial.println();

  Serial.print("Motor Voltage Readings: ");
    for (int i = 0; i < 4; i ++){
      currentCurrent[i] = analogRead(currentPin[i]);
      Serial.print(currentCurrent[i]);
      Serial.print(" ");
    }

  Serial.println();
  Serial.println();

  delay(1000);

  for (int i = 0; i < 4; i ++){
    analogWrite(pwmPin[i], 0); //OFF
  }  

  //Display on 7 SEG

  HCMAX7219.Clear();
  HCMAX7219.print7Seg("POWER", 8); //Print Text taken
  HCMAX7219.Refresh();
  delay(2000);
  for (int i = 0; i < 4; i ++){
    HCMAX7219.Clear();
    HCMAX7219.print7Seg(currentCurrent[i], 8); //Print Current
    HCMAX7219.Refresh();
    delay(2000);
  }
  HCMAX7219.Clear();
  HCMAX7219.Refresh();
  delay(2000);
}

void MecanumRover::checkIfStuck(int ticks){
  for (int i = 0; i < 4; i ++){
    if ((millis() - encoderChangeTime[i] > 2000) && (sensorStop == false)){ //Has it been long since the last time the encoder ticked? Ignore if the sensor is causing the robot to stop
      for (int v = 0; v < 4; v ++){ //Turn them all off
        analogWrite(pwmPin[v], 0); //OFF
      }
      HCMAX7219.Clear();
      HCMAX7219.print7Seg("STUCK", 8); //Print Text taken
      HCMAX7219.Refresh();
      delay(1000);

      HCMAX7219.Clear();
      HCMAX7219.print7Seg(i+1, 8); //Print which wheel is stuck
      HCMAX7219.Refresh();
      delay(1000);
      counter[i] == abs(ticks); //Make counter overflow so movement is cancelled
    }
    else {
      encoderChangeTime[i] = millis();
    }
  }  
}

void MecanumRover::strafe(int ticks){ //Move forward/backward a number of ticks
  strafing = true;
  if (ticks > 0){ //Right strafe
    digitalWrite(directionPin[0], LOW);
    digitalWrite(directionPin[1], HIGH);
    digitalWrite(directionPin[2], LOW);
    digitalWrite(directionPin[3], HIGH);
  }
  else { //Left strafe

    digitalWrite(directionPin[0], HIGH);
    digitalWrite(directionPin[1], LOW);
    digitalWrite(directionPin[2], HIGH);
    digitalWrite(directionPin[3], LOW);
  }

  startTime = millis();

  /*
  Strafing only works with all 4 wheels
  */

  for (int i = 0; i < 4; i ++){
    counter[i] = 0; //Initialise encoder count
    analogWrite(pwmPin[i], baseSpeed);
  }

  while(counter[0] + counter[1] + counter[2] + counter[3] < abs(ticks) * 4){ //While total sum of encoders for 2 wheels is less than the desired distance x4
    adjustSpeed(ticks);
    //checkIfStuck(abs(ticks)); //Check for stuck, if stuck then break out
  }  
    
  for (int i = 0; i < 4; i ++){ //For all 4 wheels
    analogWrite(pwmPin[i], 0); //Off
  }
  brake();
  strafing = false;
  printResults(ticks);
}

void MecanumRover::turn(int ticks){ //Move forward/backward a number of ticks
  if (-ticks > 0){ //CW
    digitalWrite(directionPin[0], HIGH);
    digitalWrite(directionPin[1], LOW);
    digitalWrite(directionPin[2], LOW);
    digitalWrite(directionPin[3], HIGH);
  }
  else { //CCW
    digitalWrite(directionPin[0], LOW);
    digitalWrite(directionPin[1], HIGH);
    digitalWrite(directionPin[2], HIGH);
    digitalWrite(directionPin[3], LOW);
  }

  startTime = millis();

  for (int i = 0; i < 4; i ++){
    counter[i] = 0; //Initialise encoder count
    analogWrite(pwmPin[i], baseSpeed);
  }

  while(counter[0] + counter[1] + counter[2] + counter[3] < abs(ticks) * 4){ //While total sum of encoders for all wheels is less than the desired distance x4
    adjustSpeed(ticks);
    //checkIfStuck(abs(ticks)); //Check for stuck, if stuck then break out
  }  
    
  for (int i = 0; i < 4; i ++){ //For all 4 wheels
    analogWrite(pwmPin[i], 0); //Off
  }
  brake();
  printResults(ticks);
}

void MecanumRover::printResults(int ticks){
  long endTime = millis() - startTime;
  delay(500);
  //Print results
  Serial.println();
  Serial.print("Length: ");
  Serial.print(ticks);
  Serial.println(" ticks");

  Serial.print("Encoder readings: ");
  for (int i = 0; i < 4; i ++){
    Serial.print(counter[i]);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("Motor Voltage Readings: ");
  for (int i = 0; i < 4; i ++){
    Serial.print(currentCurrent[i]);
    Serial.print(" ");
  }

  Serial.println();
  Serial.print("Time of movement sequence: ");
  Serial.print(endTime);
  Serial.println(" milliseconds");
  Serial.println();

  HCMAX7219.Clear();
  HCMAX7219.print7Seg(endTime, 8); //Print Time taken
  HCMAX7219.Refresh();
}

void MecanumRover::rightDiagonal(int ticks){ //Move forward/backward a number of ticks
  strafing = true;

  if (ticks > 0){ //Right strafe
    digitalWrite(directionPin[0], LOW);
    digitalWrite(directionPin[1], HIGH);
    digitalWrite(directionPin[2], LOW);
    digitalWrite(directionPin[3], HIGH);
  }
  else { //Left strafe

    digitalWrite(directionPin[0], HIGH);
    digitalWrite(directionPin[1], LOW);
    digitalWrite(directionPin[2], HIGH);
    digitalWrite(directionPin[3], LOW);
  }

  startTime = millis();

  for (int i = 0; i < 4; i ++){
    counter[i] = 0; //Initialise encoder count
  }
  analogWrite(pwmPin[0], baseSpeed);
  analogWrite(pwmPin[3], baseSpeed);

  while(counter[0] + counter[1] + counter[2] + counter[3] < abs(ticks) * 2){ //While total sum of encoders for 2 wheels is less than the desired distance x4
    adjustSpeed(ticks/2); //Half as we using half the wheels
    //checkIfStuck(abs(ticks)); //Check for stuck, if stuck then break out
  }  
    
  for (int i = 0; i < 4; i ++){ //For all 4 wheels
    analogWrite(pwmPin[i], 0); //Off
  }
  strafing = false;
  printResults(ticks);
}

void MecanumRover::brake(){
  for (int i = 0; i < 4; i ++){ //For all 4 wheels
    analogWrite(pwmPin[i], 255); //On
  }
  for (int i = 0; i < 4; i ++){
    digitalWrite(directionPin[i], HIGH);
  }
  delay(brakeTime);
  for (int i = 0; i < 4; i ++){
    digitalWrite(directionPin[i], LOW);
  } 
  delay(brakeTime);
  for (int i = 0; i < 4; i ++){
    digitalWrite(directionPin[i], HIGH);
  }  
  for (int i = 0; i < 4; i ++){ //For all 4 wheels
    analogWrite(pwmPin[i], 0); //Off
  }
}