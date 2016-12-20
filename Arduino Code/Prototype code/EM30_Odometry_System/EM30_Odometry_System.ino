// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// %%%%%%%%%%%%% Odemetry contorller for EMG30 Motors %%%%%%%%%%%%%
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// © UBRobotics 2016
// Written by Ed Chamberlain
//
// This sketch requires an arduino board with 4+ inturrupts such as
// the 101, Mega, zero, Leonardo, Micro, Due.
//
// Do NOT accidently supply 12V to the Arduino - you will break it.
// You can use the 5V supply for the encoders, remeber to connect
// grounds. Pull encoder signals high using 4K7 - 10K resistors
//
// EMG30 Wiring as follows:
//   Wire colour |  Connection
//   ------------|------------------
//    1 Purple   |  Hall Sensor B Vout
//    2 Blue     |  Hall sensor A Vout
//    3 Green    |  Hall sensor ground
//    4 Brown    |  Hall sensor Vcc
//    5 Red      |  + Motor
//    6 Black    |  - Motor
//
// Co-ords reference centrepoint of robot and orientation
// references forward direction.

// ----------------------------------------------------------------
// ----------------------- Global Variables -----------------------
// ----------------------------------------------------------------

// define vector struct
struct vector {
  float x;
  float y;
  float o;
};

// Set initial position here:
vector pos = {0, 0, 0};

// Set wheel base and wheel diamter here:
static const int wheelBase = 200;
static const int wheelDiam = 100;

// Set which pin the wheel encoders are connected to. These must
// be inturrpt enabled. Ensure encoders are pulled up to VCC
// using suitably selected resistors.
#define LeftEnc1 1
#define LeftEnc2 2
#define RightEnc1 3
#define RightEnc2 4

#define pi 3.14159
float wheelStep;
float angleStep;
float leftx, rightx, lefty, righty; // call compWheelPos() before

vector travel;

// ----------------------------------------------------------------
// ------------------------- Program Loop -------------------------
// ----------------------------------------------------------------

void setup() {
  // init serial
  Serial.begin(9600);
  Serial.println("Running");
  Serial.println("SETUP: Serial init");

  // init inturrpts
  attachInterrupt(digitalPinToInterrupt(LeftEnc1), ISRLeftEnc1,
                  FALLING);
  attachInterrupt(digitalPinToInterrupt(LeftEnc2), ISRLeftEnc2,
                  FALLING);
  attachInterrupt(digitalPinToInterrupt(RightEnc1), ISRRightEnc1,
                  FALLING);
  attachInterrupt(digitalPinToInterrupt(RightEnc2), ISRRightEnc2,
                  FALLING);
  Serial.println("SETUP: ISR init");

  //compute global variables
  wheelStep = 100 * pi / 180;
  angleStep = rad2deg(wheelStep / wheelBase);

  // compute step vector
  travel.x = wheelBase / 2 * sin(deg2rad(angleStep));
  travel.y = wheelBase / 2 - (wheelBase / 2) *
             cos(deg2rad(angleStep));

  // debug variables
  Serial.println("SETUP: Global Values init");
  Serial.print("NOTE: wheelStep = ");
  Serial.println(angleStep);
  Serial.print("NOTE: angleStep = ");
  Serial.println(angleStep);
  Serial.print("NOTE: travel.x = ");
  Serial.println(travel.x);
  Serial.print("NOTE: travel.y = ");
  Serial.println(travel.y);
}

void loop() {
  // put your main code here, to run repeatedly:

}

// ----------------------------------------------------------------
// -------------------------- Functions ---------------------------
// ----------------------------------------------------------------

void compWheelPos () {
  leftx = pos.x + (sin(deg2rad(pos.o + 90)) * wheelBase / 2);
  rightx = pos.x + (sin(deg2rad(pos.o - 90)) * wheelBase / 2);
  lefty = pos.y + (cos(deg2rad(pos.o + 90)) * wheelBase / 2);
  righty = pos.y + (cos(deg2rad(pos.o - 90)) * wheelBase / 2);
}

float deg2rad (float deg) {
  float rad = (deg / 360) * (2 * pi);
  return rad;
}

float rad2deg (float rad) {
  float deg = (rad / (2 * pi)) * (360);
  return deg;
}

vector rotateVector(vector input, float pheta) {
  vector output;

  output.x = input.x * cos(deg2rad(pheta)) - input.y *
             sin(deg2rad(pheta));
  output.y = input.y * sin(deg2rad(pheta)) - input.x *
             cos(deg2rad(pheta));
  output.o = pheta;

  return output;
}

void leftEnc () {
  vector rotated = rotateVector(travel, pos.o);
  pos.x = pos.x + rotated.x;
  pos.y = pos.y - rotated.y;

  pos.o = pos.o - angleStep;
}

void rightEnc () {
  vector rotated = rotateVector(travel, pos.o);
  pos.x = pos.x + rotated.x;
  pos.y = pos.y + rotated.y;

  pos.o = pos.o + angleStep;
}

// ----------------------------------------------------------------
// ----------------- Interrupt Service Routines -------------------
// ----------------------------------------------------------------

void ISRLeftEnc1() {
  leftEnc();
}

void ISRLeftEnc2() {
  leftEnc();
}

void ISRRightEnc1() {
  rightEnc();
}

void ISRRightEnc2() {
  rightEnc();
}
