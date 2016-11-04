//UB ROBOTICS EUROBOT 2016, DANBOT

// Values of 0 being sent using serial.write() have to be cast as a byte to stop them being misinterpereted as NULL
// This is a bug with arduino 1
#include <Servo.h>

int ultrasoundPin =  7;  //????

Servo armServo;
Servo parasolServo;

#define CMD                 (byte)0x00              //  MD25 command byte of 0

#define WRITESP1            0x31                    // Byte writes speed to motor 1
#define TURN                0x32
#define WRITEACCEL          0x33                    // Byte writes a value of acceleration
#define RESETREG            0x35                    // Byte resets encoders
#define SETMODE             0x34                    // Byte sets mode
#define READIVS             0x2C                    // Byte reads motor currents and battery voltage        
#define READENCS            0x25                    // Byte reads both encoders
#define GET_VER             0x29

#define LCD_RX              0x02                    // RX and TX pins used for LCD0303 serial port
#define LCD_TX              0x03
#define LCD03_HIDE_CUR      0x04
#define LCD03_CLEAR         0x0C
#define LCD03_SET_CUR       0x02

int mode = 11;   // choose the input pin (for a pushbutton)
int side = 0;     // variable for reading the pin status

void setup() {
  Serial.begin(38400);

  armServo.attach(8);
  armServo.write(22);

  Serial.write(CMD);                                            // Set MD25 accelleration value
  Serial.write(WRITEACCEL);
  Serial.write(10);
  delayMicroseconds(10);                                        // Wait for this to be processed
  Serial.write(CMD);                                            // Reset the encoder registers to 0
  Serial.write(RESETREG);
  Serial.write(CMD);                                            // Set mode to 2, Both motors controlled by writing to speed 1
  Serial.write(SETMODE);
  Serial.write(3);
  pinMode(mode, INPUT);    // declare pushbutton as input
}

void loop() {
  side = digitalRead(mode);
  if (side == HIGH) { //LEFTSIDE
    botArm(1);
    delay(50);

    botMove(-1300, 30, 1);
    delay(50);

    botMove(1500, 30, 1);
    delay(50);

    //botTurn(-90, 10);   //initial turn
    //delay(50);

    botMove(-320, 20, 1);
    delay(50);

    botTurn(90, 10);
    delay(50);

    botMove(600, 20, 2); //Towards Hut A
    delay(50);

    botMove(900, 20, 0); //Towards Hut A
    delay(50);

    botMove(-300, 20, 0);
    delay(50);

    botTurn(-90, 10);
    delay(50);

    botMove(-430, 20, 1);
    delay(50);

    botTurn(90, 10);
    delay(50);

    botMove(400, 20, 0);  //Towards Hut B
    delay(50);

    botMove(-1700, 20, 0);
    delay(50);

    botTurn(-45, 10);
    delay(50);

    botTurn(45, 10);
    delay(50);

    botMove(-250, 20, 0);
    delay(50);

    botTurn(90, 10);
    delay(50);

    botMove(-500, 20, 1);
    delay(50);

    botTurn(90, 10);
    delay(50);

    botMove(-700, 20, 1);
    delay(15);

    botTurn(-45, 10);
    delay(50);

    botMove(-50, 10, 1);
    delay(50);

    botMove(50, 10, 1);
    delay(50);

    armServo.detach();

    while (millis() < 90000) {
      botSpeed(0);
    }
    parasolServo.attach(5);
    parasolServo.write(115);
    botSpeed(0);
    delay(600000);

  } else {              //OTHER SIDE!
    botArm(1);
    delay(50);


    botMove(-1300, 30, 1);
    delay(50);

    botMove(1500, 30, 1);
    delay(50);

    //botTurn(-90, 10);   //initial turn
    //delay(50);

    botMove(-320, 20, 1);
    delay(50);

    botTurn(-90, 10);
    delay(50);

    botMove(600, 20, 2); //Towards Hut A
    delay(50);

    botMove(700, 20, 0); //Towards Hut A
    delay(50);

    botMove(-300, 20, 0);
    delay(50);

    botTurn(90, 10);
    delay(50);

    botMove(-430, 20, 1);
    delay(50);

    botTurn(-90, 10);
    delay(50);

    botMove(400, 20, 0);  //Towards Hut B
    delay(50);

    botMove(-1700, 20, 0);
    delay(50);

    botTurn(45, 10);
    delay(50);

    botTurn(-45, 10);
    delay(50);

    botMove(-250, 20, 0);
    delay(50);

    botTurn(-90, 10);
    delay(50);

    botMove(-500, 20, 1);
    delay(50);

    botTurn(-90, 10);
    delay(50);

    botMove(-700, 20, 1);
    delay(15);

    botTurn(45, 10);
    delay(50);

    botMove(-50, 10, 1);
    delay(500);

    botMove(50, 10, 1);
    delay(500);

    armServo.detach();

    while (millis() < 90000) {
      botSpeed(0);
    }
    parasolServo.attach(5);
    parasolServo.write(115);
    botSpeed(0);
    delay(600000);

  }
}

//---------------------------------------------------------------------------------------

long readEncoder1() {                       // Function to read and display the value of both encoders, returns value of first encoder
  long result1 = 0;
  long result2 = 0;
  Serial.write(CMD);
  Serial.write(READENCS);
  while (Serial.available() < 8) {}        // Wait for 8 bytes, first 4 encoder 1 values second 4 encoder 2 values
  result1 = Serial.read();                 // First byte for encoder 1, HH.
  result1 <<= 8;
  result1 += Serial.read();                // Second byte for encoder 1, HL
  result1 <<= 8;
  result1 += Serial.read();                // Third byte for encoder 1, LH
  result1 <<= 8;
  result1  += Serial.read();               // Fourth byte for encoder 1, LL
  result2 = Serial.read();
  result2 <<= 8;
  result2 += Serial.read();
  result2 <<= 8;
  result2 += Serial.read();
  result2 <<= 8;
  result2 += Serial.read();
  Serial.println(result1);
  return result1;
}

long readEncoder2() {                       // Function to read and display the value of both encoders, returns value of first encoder
  long result1 = 0;
  long result2 = 0;
  Serial.write(CMD);
  Serial.write(READENCS);
  while (Serial.available() < 8) {}        // Wait for 8 bytes, first 4 encoder 1 values second 4 encoder 2 values
  result1 = Serial.read();                 // First byte for encoder 1, HH.
  result1 <<= 8;
  result1 += Serial.read();                // Second byte for encoder 1, HL
  result1 <<= 8;
  result1 += Serial.read();                // Third byte for encoder 1, LH
  result1 <<= 8;
  result1  += Serial.read();               // Fourth byte for encoder 1, LL
  result2 = Serial.read();
  result2 <<= 8;
  result2 += Serial.read();
  result2 <<= 8;
  result2 += Serial.read();
  result2 <<= 8;
  result2 += Serial.read();
  Serial.println(result2);
  return result2;
}

void botSpeed(int speed) {
  Serial.write(CMD);
  Serial.write(WRITESP1);
  Serial.write(speed);
  Serial.write(CMD);
  Serial.write(TURN);
  Serial.write(0);
}

void botTurn(int angle, int velocity) {
  Serial.println("botTurn");
  int steps = ((angle * 2 * 3.14159 * 132) / 360) * ((100 * 3.14159) / 360);
  long initEncValue = readEncoder1();
  long encValue = initEncValue;
  int y;

  Serial.println(steps);

  if (angle > 0) {
    y = velocity;

    while (encValue < (steps + initEncValue)) {
      Serial.write(CMD);
      Serial.write(WRITESP1);
      Serial.write(0);
      Serial.write(CMD);
      Serial.write(TURN);
      Serial.write(y);
      encValue = readEncoder1();
    }
  } else if (angle < 0) {
    y = (-1) * velocity;

    while (encValue > (initEncValue + steps)) {
      Serial.write(CMD);
      Serial.write(WRITESP1);
      Serial.write(0);
      Serial.write(CMD);
      Serial.write(TURN);
      Serial.write(y);
      encValue = readEncoder1();
    }
  }
  Serial.write(CMD);
  Serial.write(WRITESP1);
  Serial.write(0);
  Serial.write(CMD);
  Serial.write(TURN);
  Serial.write(0);
}

void botMove (long dist, int velocity, bool sense) {

  Serial.println("botMove");
  long initEncValue = readEncoder1();
  long encValue = initEncValue;
  long steps = dist * ((100 * 3.14159) / 360);
  bool state1 = 0;
  bool state2 = 0;

  Serial.println(steps);
  if (dist > 0) {
    while (encValue < (steps + initEncValue)) {
      state1 = digitalRead(7);
      state2 = digitalRead(9);
      Serial.print("STATE1: ");
      Serial.print(state1);
      Serial.print("STATE2: ");
      Serial.println(state2);

      if (((state1 == 0) || (sense == 0)) || ((sense == 2 && state2 == 0))) {
        botSpeed(velocity);
        encValue = readEncoder1();
        delay(10);
      } else {
        botSpeed(0);
        delay(10);
      }
    }
  } else if (dist < 0) {
    while (encValue > (initEncValue + steps)) {
      state1 = digitalRead(7);
      state2 = digitalRead(9);
      Serial.print("STATE1: ");
      Serial.print(state1);
      Serial.print("STATE2: ");
      Serial.println(state2);
      if (((state1 == 0) || (sense == 0)) || (sense == 2 && state2 == 0)) {
        botSpeed((-1)*velocity);
        encValue = readEncoder1();
        delay(10);
      } else {
        botSpeed(0);
        delay(10);
      }
    }
  }
  botSpeed(0);
}



void botTurnOnWheel(int angle) {
  int x = 0;
  if (angle == 45) {
    x = 22;
  } else if (angle == 90) {
    x = 44;
  } else {
    x = (angle * 0.477777778) + 1;
  }

  for (int n = 0; n < x; n++) {
    Serial.write(CMD);
    Serial.write(WRITESP1);
    Serial.write(10);
    Serial.write(CMD);
    Serial.write(TURN);
    Serial.write(10);
  }
}

void botArm(int pos) {
  if (pos == 0) {
    for (pos = 160; pos >= 22; pos -= 1) {
      armServo.write(pos);
      delay(25);
    }
  } else if (pos == 1) {
    for (pos = 22; pos <= 160; pos += 1) {
      armServo.write(pos);
      delay(25);
    }
  }
}
