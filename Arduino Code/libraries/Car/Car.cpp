/*
DAGU ROVER 5 Arduino Class created by Yuya Takeuchi

Do not distribute, only for use in UBRobotics (University of Birmingham Robotics Club) projects

2016 UNIVERSITY OF BIRMINGHAM ROBOTICS CLUB
*/

//Check that the below libraries are installed:

#include "Car.h"
#include "Arduino.h"
#include "Madgyro.h"

/*
WIRING THE ROVER 5

1. Make sure the Motor Controller has power (VS and GND connected from battery)
2. Connect Left track to Channel 1, Right to Channel 2
3. Take any TWO of the Digital Pins from the Arduino and connect to the controller's CH1 Direction and CH2 Direction headers. Make note of the pin numbers.
4. Refer to Rover 5 manual online for schematic of its rotary encoders
5. Connect the Left and Right encoder jumpers to the motor controller
6. Take any TWO analog out (PWM) pin from the Arduino and connect to the controller's CH1 PWM and CH2 PWM headers. Make note of the pin numbers
OPTIONAL: Connect a buzzer across any digital pin and GND
7: Connect Pins 2 and 3 to the INT pins of CH1 and CH2 of the motor controller and make note of the pins.
https://www.arduino.cc/en/Reference/AttachInterrupt

The pin numbers you noted will be declared when you make an instance of the Car class. For example:

	Car car(22, 24, 2, 3, 4, 5, 6);

Where the numbers correspond to the pins noted. The meaning of the pins are explained below, in that order
*/

Car::Car(int direction0, int direction1, int interrupt0, int interrupt1, int pwm0, int pwm1, int buzzer, int turns, int ping0, int echo0) {

/*
Variable Declarations

direction0 = Left Track Direction Pin DIGITAL
direction1 = Right Track Direction Pin DIGITAL

interrupt0 = Interrupt Pin for Left Track DIGITAL
interrupt1 = Interrupt Pin for Right Track DIGITAL

pwm0 = Left Track PWM Pin ANALOG OUT
pwm1 = Right Track PWM Pin ANALOG OUT

buzzer = Pin for diagnostic buzzer DIGITAL (If no buzzer just set to unused digital pin)

turningSpeed = Turning PWM speed (0 - 255)
*/


//Now initialise all PIN NUMBERS to a variable
trigPin = ping0; // Ultrasound Ping Pin
echoPin = echo0;

directionPin[0] = direction0;
directionPin[1] = direction1;

interruptPin[0] = interrupt0;
interruptPin[1] = interrupt1;

pwmPin[0] = pwm0;
pwmPin[1] = pwm1;

pinMode(directionPin[0], OUTPUT);
pinMode(directionPin[1], OUTPUT);

pinMode(interruptPin[0], INPUT);
pinMode(interruptPin[1], INPUT);

//Set Encoder Pin States
digitalWrite(interruptPin[0], LOW);
digitalWrite(interruptPin[1], LOW);

//Test
buzzerPin = buzzer;

	if (abs(turns) > 255){ //Limit of turning speed is 255
		turns = 255;
	}

turningSpeed = abs(turns);
}



void Car::displayCounters() {
//Display encoder count to serial monitor
Serial.print("Left: ");
Serial.print(counter[1]);
Serial.print(", Right: ");
Serial.println(counter[0]);
}

void Car::turn(int ticks) { //Method for turning car. 'ticks' defines how many steps it will turn for (calibration required to convert to angle)
isRunning = true; //Car is running

counter[0] = 0; //Reset encoder counter

	if (ticks < 0){ //If number of steps is NEGATIVE, turn CLOCKWISE (set direction pins as required)
		digitalWrite(directionPin[0], HIGH);
		digitalWrite(directionPin[1], LOW);
	}
	else if (ticks > 0){ //If number of steps is POSITIVE, turn ANTICLOCKWISE (set direction pins as required)
		digitalWrite(directionPin[0], LOW);
		digitalWrite(directionPin[1], HIGH);
	}

	/*
	The following code turns the wheels at maximum speed. Experiment with analogWrite() at different speeds of each track to get the smoothest turning
	*/

	digitalWrite(pwmPin[0], HIGH);
	digitalWrite(pwmPin[1], HIGH);

	
	while (counter[0] < abs(ticks)){ //While the magnitude of ticks is below the required number of steps, stay in loop
		delayMicroseconds(1);
	}

	//Stop tracks, they have moved sufficient number of steps

	digitalWrite(pwmPin[0], LOW);
	digitalWrite(pwmPin[1], LOW);
	Serial.print("End of Cycle.");
	isRunning = false; //Important
}

void Car::move(int distance, int speed){
	if (speed <= 0){
		digitalWrite(directionPin[0], HIGH);	
		digitalWrite(directionPin[1], HIGH);	
	}

	else { //If positive, make sure direction reverse pins are LOW on both tracks
		digitalWrite(directionPin[0], LOW);	
		digitalWrite(directionPin[1], LOW);	
	}

	//Set PWM pins to given speed
	analogWrite(pwmPin[0], abs(speed));
	analogWrite(pwmPin[1], abs(speed));

	counter[0] = 0;

	bool stopped = false;

	while (counter[0] < abs(distance)){ //While the magnitude of ticks is below the required number of steps, stay in loop
		delayMicroseconds(1);
		int sensorRead = readSensor();
		if ((sensorRead < 45)&&(sensorRead != 0)){ //IS THERE OBSTRUCTION
			analogWrite(pwmPin[0], 0); //STOP
			analogWrite(pwmPin[1], 0);
			stopped = true;
		}
		else{
			if (stopped == true){
				sensorRead = readSensor();
				if ((sensorRead < 45)&&(sensorRead != 0)){ 
					analogWrite(pwmPin[0], 0); //STOP
					analogWrite(pwmPin[1], 0);
					stopped = true;
				}
				else {
					analogWrite(pwmPin[0], abs(speed));
					analogWrite(pwmPin[1], abs(speed));
					stopped = false;
				}
			}
			else {
			analogWrite(pwmPin[0], abs(speed));
			analogWrite(pwmPin[1], abs(speed));
			stopped = false;
			}
		}
	}

	analogWrite(pwmPin[0], 0);
	analogWrite(pwmPin[1], 0);

	delay(1000);
}

void Car::stop(){ //Stop the car by bringing both PWM pins to LOW
	digitalWrite(pwmPin[0], LOW);
	digitalWrite(pwmPin[1], LOW);
}

void Car::gyroturn(int radians){ //Turn by angle radians
	//Set direction (Positive = Counter Clockwise)
	if (radians > 0){
		//CCW
		digitalWrite(directionPin[0], LOW);
		digitalWrite(directionPin[1], HIGH);
	}
	else{
		digitalWrite(directionPin[1], LOW);
		digitalWrite(directionPin[0], HIGH);
	}

	Madgyro turning; //Activate gyro

	analogWrite(pwmPin[0], turningSpeed);
	analogWrite(pwmPin[1], turningSpeed);

	while (abs(turning.readYaw()) < abs(radians)){ //While the magnitude of the GYRO YAW is less than the magnitude of REQUIRED CALIBRATED RADIANS
		if (radians > 0){ //ANTICLOCK
		delay(47); //IMPORTANT SETS SENSITIVITY (LOWER IS SMALLER)	
		}
		else {
			delay(43);
		}
	}
	//Turn off the car
	analogWrite(pwmPin[0], 0);
	analogWrite(pwmPin[1], 0);
	delay(1000);
}

int Car::calibrategyro(){
	Madgyro turning; //Activate Gyro
	tone(buzzerPin,10);
	delay(5000); //You have 5 seconds to turn the robot 90deg CCW
	rightangle = turning.readYaw(); //Calibrate raw value of a 90deg turn
	noTone(buzzerPin); //End feedback
	return rightangle;
}

long Car::readSensor() {

  long duration, cm;

pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);

  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);
  
  return(cm);
  delay(100);
}
/*

Following snippet of code is neccesary for the interrupt to work properly. Always include in your sketches.

*/
long Car::microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
