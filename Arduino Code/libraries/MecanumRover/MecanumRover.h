#ifndef MecanumRover_h
#define MecanumRover_h

#include "Arduino.h"
#include "NewPing.h"

class MecanumRover
{
  public:
    MecanumRover(int dir0, int dir1, int dir2, int dir3, int pwm0, int pwm1, int pwm2, int pwm3, int int0, int int1, int int2, int int3, int cur0, int cur1, int cur2, int cur3, int base, float factor, int buzzerPin); //Constructor
    void adjustSpeed(int ticks); //Used internally to adjust individual wheel speeds
    void strafe(int ticks); //Strafe Left Right (Right Positive)
    void rightDiagonal(int ticks); //Experimental do not use
    void turn(int ticks); //Turn (CCW positive)
    void move(int ticks, NewPing frontSensor, NewPing rearSensor);
    void testHardware(); //Test of motors and encoder
    float correctionFactor; //Linear correction gain of wheels (Lower, motor may take longer to normalise speeds, higher and the levels will flutctuate and jitter the robot)
    int baseSpeed; //Base speed of movement
	int interruptPin[4]; //Interrupt pins of Arduino for 4 wheels
    int counter[4]; //Counter of encoder
    int encoderChangeTime[4]; //The time in milliseconds between the start of the Arduino and the last tick of the encoder
  private:
    int speedDeltaCap = 10; //Maximum difference in PWM adjustment when correcting straightness
    int pwmPin[4]; //PWM pins of controller for speed of all 4 wheels
    int directionPin[4]; //Digital pins of controller for direction for 4 wheels
    int currentPin[4]; //Analog Input pins for reading motor current
    int currentCurrent[4]; //What is the motor current now?
    int buzzer; //Pin of buzzer
    bool sensorStop = false; //Has the sensor stopped the robot?
    void printResults(int ticks);
    void brake();
    void checkIfStuck(int ticks);   //Is the robot stuck somewhere whilst moving?
};

#endif