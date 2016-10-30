//UB ROBOTICS EUROBOT 2016, DANBOT
//All code by Daniel Hingston
//Prepared for National Round at Middlesex University, 21/04/16

//LEFT SIDE, LEFT SIDE, LEFT SIDE, LEFT SIDE, LEFT SIDE, LEFT SIDE

//PURPLE PURPLE PURPLE PURPLE PURPLE PURPLE PURPLE PURPLE PURPLE

#include <SoftwareSerial.h>
#include <Servo.h>
#include <NewPing.h>
Servo myservo;

// Values of 0 being sent using serial.write() have to be cast as a byte to stop them being misinterpereted as NULL
// This is a bug with arduino 1
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

SoftwareSerial lcd03 = SoftwareSerial(LCD_RX, LCD_TX);          // Define the serial port for the LCD03

long encValue = 0;
byte softwareRev = 0;
int counter = 0;
long elapsed = 0;
int pos = 30;
int distance = 0;
int distancef = 0;
int limitb = 30;
int limitf = 10;

NewPing ping(5,6,50);
NewPing ping1(3,4,50);

void setup(){
  Serial.begin(38400);
  lcd03.begin(9600);
  myservo.attach(10);
  
  Serial.write(CMD);                                            // Set MD25 accelleration value
  Serial.write(WRITEACCEL);
  Serial.write(10);
  delayMicroseconds(10);                                        // Wait for this to be processed
  Serial.write(CMD);                                            // Reset the encoder registers to 0
  Serial.write(RESETREG);         
  Serial.write(CMD);                                            // Set mode to 2, Both motors controlled by writing to speed 1
  Serial.write(SETMODE);
  Serial.write(3);    
  
  Serial.write(CMD);                                            // Get software version of MD25
  Serial.write(GET_VER);
  while(Serial.available() < 1){}                               // Wait for byte to become available         
  softwareRev = Serial.read();  
  
  lcd03.write(LCD03_CLEAR);                                     // Clear the LCD03 screen
  lcd03.write(LCD03_HIDE_CUR);                                  // Hide the LCD03 cursor
}

void loop(){
  pos = 115;
  myservo.write(pos);
  delay(2000);
  counter = 0;                     
  while(counter < 29){                //Reverse out of starting area
distance = (ping.ping_cm());
delay(30);
    if ((distance != 0) && (distance < limitb)){
    Serial.write(CMD);
    Serial.write(WRITESP1);
    Serial.write(0);
    Serial.write(CMD);
    Serial.write(TURN);
    Serial.write(0);
    }
    else {
    Serial.write(CMD);
    Serial.write(WRITESP1);
    Serial.write(10);
    Serial.write(CMD);
    Serial.write(TURN);
    Serial.write(0);
    counter ++;
        }
    }
  delay(2000);
  counter = 0;
  while(counter < 43){                //Danbot turn 90' to the left
    Serial.write(CMD);
    Serial.write(WRITESP1);
    Serial.write(0);
    Serial.write(CMD);
    Serial.write(TURN);
    Serial.write(-10);
    counter ++;
  }
  delay(2000);
  counter = 0;                  
  limitb = 9;   
  while(counter < 215){                //Reverse back into Hut A
distance = (ping.ping_cm());
delay(30);
    if ((distance != 0) && (distance < limitb)){
    Serial.write(CMD);
    Serial.write(WRITESP1);
    Serial.write(0);
    Serial.write(CMD);
    Serial.write(TURN);
    Serial.write(0);
    }
    else {
    Serial.write(CMD);
    Serial.write(WRITESP1);
    Serial.write(10);
    Serial.write(CMD);
    Serial.write(TURN);
    Serial.write(0);
    counter ++;
        }
    }
  delay(2000);
  counter = 0;                     
  while(counter < 28){                //Move forwards small distance
    distancef = (ping1.ping_cm());
    delay(30);
    if ((distancef != 0) && (distancef < limitf)){ //STOP
    Serial.write(CMD);
    Serial.write(WRITESP1);
    Serial.write(0);
    Serial.write(CMD);
    Serial.write(TURN);
    Serial.write(0);
    }
    else {
    Serial.write(CMD);
    Serial.write(WRITESP1);
    Serial.write(-10);
    Serial.write(CMD);
    Serial.write(TURN);
    Serial.write(0);
    counter ++;
    }
  }
  delay(2000);
  counter = 0;
  while(counter < 43){                //Danbot turn 90' to the right
    Serial.write(CMD);
    Serial.write(WRITESP1);
    Serial.write(0);
    Serial.write(CMD);
    Serial.write(TURN);
    Serial.write(10);
    counter ++;
  }
  delay(2000);
  counter = 0;           
  limitb = 30;          
  while(counter < 50){                //Move backwards small distance
distance = (ping.ping_cm());
delay(30);
    if ((distance != 0) && (distance < limitb)){
    Serial.write(CMD);
    Serial.write(WRITESP1);
    Serial.write(0);
    Serial.write(CMD);
    Serial.write(TURN);
    Serial.write(0);
    }
    else {
    Serial.write(CMD);
    Serial.write(WRITESP1);
    Serial.write(10);
    Serial.write(CMD);
    Serial.write(TURN);
    Serial.write(0);
    counter ++;
        }
    }
  delay(2000);
  counter = 0;
  while(counter < 43){                //Danbot turn 90' to the left
    Serial.write(CMD);
    Serial.write(WRITESP1);
    Serial.write(0);
    Serial.write(CMD);
    Serial.write(TURN);
    Serial.write(-10);
    counter ++;
  }
  delay(2000);
  counter = 0;  
  limitb = 9;                   
  while(counter < 45){                //Move backwards small distance into door of Hut B
   distance = (ping.ping_cm());
   delay(30);
    if ((distance != 0) && (distance < limitb)){
    Serial.write(CMD);
    Serial.write(WRITESP1);
    Serial.write(0);
    Serial.write(CMD);
    Serial.write(TURN);
    Serial.write(0);
    }
    else {
    Serial.write(CMD);
    Serial.write(WRITESP1);
    Serial.write(10);
    Serial.write(CMD);
    Serial.write(TURN);
    Serial.write(0);
    counter ++;
        }
    }
  delay(2000);
  counter = 0;                     
  while(counter < 55){                //Move forwards small distance
distancef = (ping1.ping_cm());
delay(30);
    if ((distancef != 0) && (distancef < limitf)){
    Serial.write(CMD);
    Serial.write(WRITESP1);
    Serial.write(0);
    Serial.write(CMD);
    Serial.write(TURN);
    Serial.write(0);
    }
    else {
    Serial.write(CMD);
    Serial.write(WRITESP1);
    Serial.write(-10);
    Serial.write(CMD);
    Serial.write(TURN);
    Serial.write(0);
    counter ++;
        }
    }
  delay(2000);
  counter = 0;
  while(counter < 43){                //Danbot turn 90' to the right
    Serial.write(CMD);
    Serial.write(WRITESP1);
    Serial.write(0);
    Serial.write(CMD);
    Serial.write(TURN);
    Serial.write(10);
    counter ++;
  }
  delay(2000);
  counter = 0;                     
  while(counter < 45){                //Go and sulk in corner
    distancef = (ping1.ping_cm());
    delay(30);
    if ((distancef != 0) && (distancef < limitf)){ //STOP
    Serial.write(CMD);
    Serial.write(WRITESP1);
    Serial.write(0);
    Serial.write(CMD);
    Serial.write(TURN);
    Serial.write(0);
    }
    else {
    Serial.write(CMD);
    Serial.write(WRITESP1);
    Serial.write(-10);
    Serial.write(CMD);
    Serial.write(TURN);
    Serial.write(0);
    counter ++;
    }
  }
  delay(2000);
  counter = 0;
  while(counter < 1){
    elapsed = millis();
    if (elapsed>92000){
      break;
    }
    }
pos = 33;
myservo.write(pos);
delay(3600000);
}
 
