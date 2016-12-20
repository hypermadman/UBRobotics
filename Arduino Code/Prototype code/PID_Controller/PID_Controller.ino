// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// %%%%%%%%%%%%%%%%%%%%% Basic PID Controller %%%%%%%%%%%%%%%%%%%%%
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// © UBRobotics 2016
// Written by Ed Chamberlain
//
// A basic example of a PID controller using the PID library to 
// control an analog outpur from an analog input.

// ----------------------------------------------------------------
// ----------------------- Global Variables -----------------------
// ----------------------------------------------------------------

// Library includes
#include <PID_v1.h>

// define variables
double Setpoint, Input, Output;

// define input and output pins here:
#define inputPIN 1 // must be analog pin.
#define outputPIN 2

// Set PID Tuning here:
const double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
// '&' creates address pointer to the variables not a pointer as 
// created with '*'. Its important, dont change it

// ----------------------------------------------------------------
// ------------------------- Program Loop -------------------------
// ----------------------------------------------------------------

void setup() {
  //init serial
  Serial.begin(9600);
  Serial.println("Running");

  // init variables
  Input = analogRead(inputPIN);
  Setpoint = 100;

  // Turn on PID
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  Input = analogRead(inputPIN);
  myPID.Compute();
  analogWrite(outputPIN, Output);

  debug2var(Input, Output);
}

// ----------------------------------------------------------------
// -------------------------- Functions ---------------------------
// ----------------------------------------------------------------

void debug2var (double var1, double var2) {
  Serial.print(var1);
  Serial.print("  |  ");
  Serial.println(var2);
}

