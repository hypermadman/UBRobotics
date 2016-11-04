Designed for use with the DAGU Rover 5 (4 motor 4 encoder) with DAGU 4-channel motor controller.

1: Make sure VCC of the controller is connected to 5V on the Arduino before connecting the motor battery to Vbat. 

2: Encoders should be attached as follows (from left to right):

	VCC (Red), ChA (white), ChB (yellow), GND (black)

3: Above the VCC pin is the interrupt pin (INT). Connect this pin to the designated interrupt pins on Arduino (refer to Arduino references for which boards have which interrupt pins).

4: Connect any digital pins of the Arduino to the 4 direction pins (DIR) on the controller. 

5: Connect any PWM enabled pins to the 4 PWM pins on the controller. 

6: Connect any Analog In enabled pins to the 4 current pins (CUR) on the controller.

7: Connect the wires from the Rover 5 as follows: 

Motor order:

	CH1 - Front Right

	Ch2 - Front Left

	Ch3 - Rear Right

	Ch4 - Rear Left

Encoders should be connected as follows:

	J3 - Front Right

	J4 - Front Left

	J5 - Rear Right

	J6 - Rear Left

REQUIRED CODE WITHIN SKETCH:

Place the constructor at the top of the sketch:

	MecanumRover rover(3,5,7,9,2,4,6,8,18,19,20,21,0,1,2,3,100,5); //Where the numbers correspond to: 1-4 Direction Pins, 5-8 PWM Pins, 9-12 Interrupt Pins, 13-16 Current Read Pins, 17 Base Speed, 18 Correction Factor 

Change the name of instance, pin numbers and settings as required.

Place the following within void setup():

	attachInterrupt(digitalPinToInterrupt(rover.interruptPin[0]), count0, CHANGE);
	attachInterrupt(digitalPinToInterrupt(rover.interruptPin[1]), count1, CHANGE);
	attachInterrupt(digitalPinToInterrupt(rover.interruptPin[2]), count2, CHANGE);
	attachInterrupt(digitalPinToInterrupt(rover.interruptPin[3]), count3, CHANGE);

Place the following at the bottom of the sketch:

	void count0(){
	  rover.counter[0] ++;
	}
	void count1(){
	  rover.counter[1] ++;
	}
	void count2(){
	  rover.counter[2] ++;
	}
	void count3(){
	  rover.counter[3] ++;
	}

NOTES (proposed improvements marked with *):

Explore the different functions of the MecanumRover class within the .CPP file of the library. A list of methods and properties are available in the header file.

* So far only the interrupt pins are used to read the encoders. The encoder is designed such that ChA and ChB are mixed into the interrupt output by an XOR gate. Reading individual Channels can help determine the direction of the motors, although it's unnecesary to do so since the DIR pins decide the direction of the motors anyway. 

* Perhaps linking the class to a magnetometer can provide closed loop feedback in rotations? Current technology of accelerometers are unideal for linear displacement feedback.

CUR pins provide the current level of each motor, such that void testHardware() can detect a failed or overloaded motor (failed would return 0, overloaded would return 1023 perhaps? We need to see the logic level of this!).

Incompatible with ATMEGA-328 based boards such as the Uno and Nano due to having only 2 interrupt pins instead of the required 4, but if you wanted to shrink the size, an Arduino Micro board would work (or maybe develop your own Atmel based board?).

Keep the count0, count1, count2, and count3 methods as is. Any more intensive work in those methods will cause the Arduino to miss steps of the encoder and ignore the rest of the sketch. Perhaps the above code can be simplified further with more efficient data types? Definitely avoid writing to serial in those methods.

void adjustSpeed() is a crude way of normalising motor speeds across the whole robot. It reads the encoder counts (ticks) for all motors, takes an average, and finally adjusts the PWM level of the motors up/down depending on how much above/below its encoder count is to the average. Say, for a motor having only moved 100 ticks when the average across all 4 motors is 110, the PWM is increased by the following:

	Final PWM of motor = baseSpeed (as defined in constructor) + (Average ticks of all 4 motors - ticks of motor) * correctionFactor (a constant defined by the constructor)

	Final PWM of motor = Base Speed + (110 - 100) * Correction Factor

Lower the base speed, the lower the error (due to inertia of the motor wheels and no brakes). Find the minimum speed at which the rover can move constantly and adjust the correction factor to yield the smallest error in encoder count.

For every movement the class will return a useful log in the serial at 9600 bps. Use it to diagnose any problems.

Developed by Yuya Takeuchi (comokanu@gmail.com) for UBRobotics.