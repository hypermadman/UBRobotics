#ifndef Hinge_h
#define Hinge_h

#include "Arduino.h"

class Hinge {

public:
	Hinge(int current, int direction, int sensor);
	void turnTo(int target);
private:
	int currentPin;
	int directionPin;
	int sensorPin;
};

#endif