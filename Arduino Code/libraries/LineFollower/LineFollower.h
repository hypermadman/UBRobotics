
/*
  FreqCounter.h - Library for a Frequency Counter c.
  Created by Martin Nawrath, KHM Lab3, Dec. 2008
  Released into the public domain.
*/



#ifndef LineFollower_h
#define LineFollower_h

#include "Arduino.h"

class LineFollower
{
public:
	LineFollower(int LED, int SENSOR);
	int getValue();
private:
	int sensorPin;
	int LEDpin;
};

#endif

