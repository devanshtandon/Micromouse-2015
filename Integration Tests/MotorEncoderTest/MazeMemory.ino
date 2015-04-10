/* micromouse2015.ino 
Christopher Datsikas
Devansh Tandon

Created: 03-16-2015
Last Modified: 03-22-2015
Status: In Progress
Arduino Code for Micromouse for 
2015 Brown IEEE Robotics Competition

Github: https://github.com/devanshtandon/Micromouse-2015
Upverter: https://upverter.com/chrisdats/26efdb039ba46d67/Micromouse2015/

Components:
- TB6612FNG Dual Motor Driver Carrier - https://www.pololu.com/product/713
- 75:1 Micro Metal Gearmotor HP with Extended Motor Shaft - https://www.pololu.com/product/2215
- Sharp GP2Y0A51SK0F Analog Distance Sensor 2-15cm - https://www.pololu.com/product/2450
- Magnetic Encoders 12 CPR https://www.pololu.com/product/2598

Connections:
ARD -- OTHER COMPONEENTS
D2  -- AIN2 Motor Driver
D3  -- AIN1 Motor Driver
D4  -- STBY Motor Driver
D5 (pwm) -- PWMA Motor Driver
D6 (pwm) -- PWMB Motor Driver
D7  -- BIN1 Motor Driver
D8  -- BIN2 Motor Driver

D9  -- OUTA Encoder 1
D10 -- OUTB Encoder 1

D11 -- OUTB Encoder 2 // this looks switched
D12 -- OUTA Encoder 2 // done on purpose; to make pcb easier

A0  -- J5 Sharp Distance Sensor (rightmost)
A1  -- J4 Sharp Distance Sensor
A2  -- J3 Sharp Distance Sensor
A3  -- J2 Sharp Distance Sensor
A4  -- J1 Sharp Distance Sensor (leftmost

GND -- GND // make sure all grounds are connected

MOTOR DRIVER -- ENCODERS
AO1 -- M1 Encoder 1
AO2 -- M2 Encoder 1

BO1 -- M1 Encoder 2
BO2 -- M2 Encoder 2

*/


//---------------------------------- Funcitons and Data required for Analog IR 
//Float Multi Map function
int FmultiMap(int val, int * _in, int * _out, uint8_t size)
{
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];
  if (val >= _in[size-1]) return _out[size-1];

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > _in[pos]) pos++;

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}


// out[] holds the values wanted in mm
int out[] = {
  300, 210, 190, 150, 140, 130, 120, 110, 100, 90, 80, 70, 60, 50, 40, 30, 25, 20};
// in[] holds the measured analogRead() values for defined distances
int in[]  = {
  72, 96, 100, 113, 116, 125, 129, 132, 137, 141, 148, 159, 181, 212, 256, 320, 360, 423};

//-----------------------------------------------------------


// Define pins for Motor Driver TB6612FNG
#define PWMA 5
#define PWMB 6

#define AIN2 2
#define AIN1 3
#define BIN1 7
#define BIN2 8

#define STBY 4


// Define directions
#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4


// IR Sensor Setup
#define LEFT_BACK (4)
#define LEFT_FRONT (3)
#define FRONT (2)
#define RIGHT_FRONT (1)
#define RIGHT_BACK (0)
#define READ_SENSOR(c) (FmultiMap(analogRead(c), in, out, 18))
double sensorValues[5];


// MOTOR CONTROL CONSTANTS: NEEDS FINE-TUNING
const int COUNTS_PER_CM=93; 
const int SQUARE=1750;
const int TURN=820; 
double sp = 75;


// PID Setup
#include <PID_v1.h>
double setpoint = 0, input, output = 0;
long previousMillis = 0;
PID myPID(&input, &output, &setpoint,.15,.10,.05, DIRECT);
boolean pidSwitch=AUTOMATIC;


// Encoder Setup
#include <PololuWheelEncoders.h>
PololuWheelEncoders encoders;
double enCountsL; // encoder counts
double enCountsR;
#define WALL_THRESHOLD 60


// Maze Memory Setup
#define NORTH      1
#define EAST       2
#define SOUTH      4
#define WEST       8

// maze storage
#define NUMCELLS 256
unsigned char maze[NUMCELLS]; // array holding wall info for each cell
unsigned char mapx[NUMCELLS]; // array holding distance value of each cell
unsigned char possibleMoves[4]; // array holding possible moves from given square

unsigned char i, j = 0; // i is what cell robot is in (from 0 to 255)

unsigned char state = NORTH;

boolean frontWall;
boolean rightWall;
boolean leftWall;

boolean northWall;
boolean eastWall;
boolean westWall;
boolean southWall;



void setup() {
  
  Serial.begin(9600);
  PololuWheelEncoders::init(10,9,12,11);

  // set all the Motor Driver pins to OUTPUT
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);  // turns motor driver on

  // set up the PID
  myPID.SetSampleTime(100);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetMode(pidSwitch);

  Serial.println ("SETUP COMPELETE");
  delay(2000);
}


void loop() {
  
  wallFollow();
  // turnLeft();
  // delay(2000);

//   forwardOneSquare();
//   delay(2000);
//   forwardOneSquare();
//   delay(2000);
//   forwardOneSquare();
//   delay(2000);
//   forwardOneSquare();
//   delay(2000);
//   turnLeft();
//   delay(2000);
//   forwardOneSquare();
//   delay(2000);
//   turnLeft();
//   delay(2000);

//   forwardOneSquare();
// //  delay(2000);
//   forwardOneSquare();
// //  delay(2000);
//   forwardOneSquare();
// //  delay(2000);
//   forwardOneSquare();
// //  delay(2000);
//   turnLeft();
// //  delay(2000);
//   forwardOneSquare();
// //  delay(2000);
//   turnLeft();
// //  delay(2000);

}



void forwardOneSquare() {
  go(FORWARD, 1*SQUARE);
}

void backwardOneSquare() {
  go(BACKWARD, 1*SQUARE);
}

void turnRight() {
  go(RIGHT, 1*TURN);
}

void turnLeft() {
  go(LEFT, 1*TURN);
}

void turnAround() {
  go(LEFT, 2*TURN);
}

//Give a direction (FORWARD, BACKWARD, LEFT, or RIGHT)
//and the number of encoder counts to move.
void go(int direction, int counts) {

  resetEncoders();
  setMotorSpeeds(sp,sp);
  setMotorDirection(direction);

  if (direction == FORWARD) {

    int diff=0;
    int counter = 0;
    boolean wallClose=false;
    while ( (enCountsL+enCountsR)/2 <counts && !wallClose) {  //1487
      getEncoders();
      unsigned long currentMillis = millis();
      if(currentMillis - previousMillis > 20) {
        previousMillis = currentMillis; // save the last time you blinked the LED
        delay(1);
        diff += READ_SENSOR(LEFT_FRONT)-READ_SENSOR(RIGHT_FRONT);
        wallClose=checkWall();
        counter=counter+1;
        if (counter==3) {
          input=diff/3;
          diff=0;
          counter=0;
        }
      }
      if (abs(input) >50) myPID.SetMode(MANUAL);
      else myPID.SetMode(pidSwitch);

      myPID.Compute(); 
      setMotorSpeeds(constrain(sp+output,0,255),constrain(sp-output,0,255));
    }   

    stopRobot();

  }

  else {

    while(abs(enCountsL)<counts) {
      enCountsL = encoders.getCountsM1();
      enCountsR = encoders.getCountsM2();
    }

  }

  stopRobot();
}


void setMotorDirection(int direction) {
  if (direction == FORWARD) {
    digitalWrite(AIN1, LOW);   // Left wheel forward
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);  // Right wheel forward
    digitalWrite(BIN2, LOW);  
  }

  else if (direction == BACKWARD) {
    digitalWrite(AIN1, HIGH); // Left wheel reverse
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);  // Right wheel reverse
    digitalWrite(BIN2, HIGH);
  }

  else if (direction == LEFT) {
    digitalWrite(AIN1, HIGH);  // Left wheel reverse
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);  // Right wheel forward
    digitalWrite(BIN2, LOW);
  }

  else if (direction == RIGHT) {
    digitalWrite(AIN1, LOW);  // Left wheel forward
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW);  // Right wheel reverse
    digitalWrite(BIN2, HIGH);
  }
}

void setMotorSpeeds(double spL, double spR) {
  analogWrite(PWMA,spL);
  analogWrite(PWMB,spR);
}

void resetEncoders() {
  encoders.getCountsAndResetM1();
  encoders.getCountsAndResetM2();
  enCountsL = 0;
  enCountsR = 0;
}

void wallFollow() {

  while(1) {
    setMotorDirection(FORWARD);
    setMotorSpeeds(sp,sp);
    int diff=0;
    int counter = 0;
    boolean wallClose=false;
    while (!wallClose) { 
      unsigned long currentMillis = millis();
      if(currentMillis - previousMillis > 20) {
        previousMillis = currentMillis; 
        delay(1);
        diff += READ_SENSOR(LEFT_FRONT)-READ_SENSOR(RIGHT_FRONT);
        wallClose=checkWall();
        counter++;
        if (counter==3) {
          input=diff/3;
          diff=0;
          counter=0;
        }
      }

      if (abs(input) >50) myPID.SetMode(MANUAL);
      else myPID.SetMode(pidSwitch);

      myPID.Compute(); 
      setMotorSpeeds(constrain(sp+output,0,255),constrain(sp-output,0,255));
    }

    stopRobot();
    getSensors();
    if (sensorValues[LEFT_FRONT] > 60 && sensorValues[LEFT_BACK] > 60)
      turnLeft();
    else if (sensorValues[RIGHT_FRONT] > 60 && sensorValues[RIGHT_BACK] > 60)
      turnRight();
    else
      turnAround();
  }

}


// stop both motors and bring robot to a stop
void stopRobot() {
  digitalWrite(AIN1, HIGH);  // stop left wheel
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);  // stop right wheel
  digitalWrite(BIN2, HIGH);  
}



void getSensors() {

  sensorValues[LEFT_BACK] = READ_SENSOR(LEFT_BACK);
  sensorValues[LEFT_FRONT] = READ_SENSOR(LEFT_FRONT);
  sensorValues[FRONT] = READ_SENSOR(FRONT);
  sensorValues[RIGHT_FRONT] = READ_SENSOR(RIGHT_FRONT);
  sensorValues[RIGHT_BACK] = READ_SENSOR(RIGHT_BACK);

}


void getEncoders(){
  enCountsL=encoders.getCountsM1();
  enCountsR=encoders.getCountsM2();
}

boolean checkWall() {
  if(READ_SENSOR(FRONT) < 37)
    return true;
  else
    return false;
}



void detectWalls() {
  if (sensorValues[FRONT] < WALL_THRESHOLD) {
    frontWall= true;
     Serial.println("frontWall= true;");
  }
  else {
    frontWall=false;
  }

  if (sensorValues[LEFT_FRONT] < WALL_THRESHOLD) {
    leftWall= true;
    Serial.println("leftWall= true;");
  }
  else {
    leftWall=false;
  }

  if (sensorValues[RIGHT_FRONT] < WALL_THRESHOLD) {
    rightWall=true;
    Serial.println("rightWall= true;");
  }
  else {
    rightWall=false;
  }

if (state == NORTH) {
    northWall = frontWall;
    eastWall = rightWall;
    westWall = leftWall;
    southWall = false;
  } else if (state == SOUTH) {
    southWall = frontWall;
    eastWall = leftWall;
    westWall = rightWall;
    frontWall = false;
  } else if (state == WEST) {
    westWall = frontWall;
    northWall = rightWall;
    southWall = leftWall;
    eastWall = false;
  } else if (state == EAST) {
    eastWall = frontWall;
    northWall = leftWall;
    southWall = rightWall;
    westWall = false;
  }
}
