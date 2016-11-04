#ifndef ColorSense_h
#define ColorSense_h

#include "Arduino.h"
//define methods

class ColorSense
{
public:
	ColorSense(int RED, int GREEN, int BLUE); //pins
	//methods
	void checkBalance();
	void checkColour();
	void printColour();
	void setBalance();
	void getReading(int times);
	int getRedValue();
    int getGreenValue();
    int getBlueValue();
	
	//floats to hold colour arrays
	float colourArray[3];
	float whiteArray[3];
	float blackArray[3];

	int ledArray[3]; //define colour sensor LED pins
private:
	//instance-specific shit
	bool balanceSet;
	int red;
	int green;
	int blue;
	int avgRead;
};


#endif