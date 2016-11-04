#ifndef Madgyro_h
#define Madgyro_h

#include "CurieIMU.h"
#include "MadgwickAHRS.h"
#include "Arduino.h"

class Madgyro
{
	public:
		Madgyro();
		float readYaw();
		void calibrate();

	private:
		Madgwick filter; // initialise Madgwick object
		int ax, ay, az;
		int gx, gy, gz;
		float theyaw;
		float pitch;
		float roll;
		int factor = 800; // variable by which to divide gyroscope values, used to control sensitivity
		// note that an increased baud rate requires an increase in value of factor

		bool calibrateOffsets = false; // bool to determine whether calibration takes place or not
		const float M_PI = 3.14159; //For some reason, no constant PI exists for Arduino
};

#endif