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


// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// NEEDS MORE CALIBRATION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// when adding more values, remember to update size in the
// FmultiMap(val, _in, _out, size) function
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

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


// CONSTANTS
const float WHEEL_DIAMETER=3.12; // 3.12 cm, 31.2 mm
const float WHEEL_CIRCUM=3.141592*WHEEL_DIAMETER; //C=PI*D ~9.8018 cm
const float COUNTS_PER_REV=909.72; // 75.81 x 12
const float COUNTS_PER_CM=COUNTS_PER_REV/WHEEL_CIRCUM; // ~92.81
const float SQUARE_LENGTH=16.8;  //16.8 cm
const float COUNTS_PER_SQUARE=SQUARE_LENGTH*COUNTS_PER_CM; // 1559.24
// const float ROTATION_DIAMETER=10.3;
// const float ROTATION_CIRCUM=3.141592*ROTATION_DIAMETER;
// const float CM_PER_DEGREE=ROTATION_CIRCUM/360;
// const float COUNTS_PER_DEGREE=CM_PER_DEGREE*COUNTS_PER_CM;
// const float COUNTS_PER_90_DEGREES=COUNTS_PER_DEGREE*90;
const float WIDTH=2; //distance between center of two wheels

// NEEDS FINE-TUNING
const int INT_COUNTS_PER_CM=93; // use int for faster arithmetic
const int INT_COUNTS_PER_SQUARE=1559;
const int SQUARE=1559;
const int SQUARES=SQUARE;
const int TURN=700;
const int TURNS=TURN;


#include <PololuWheelEncoders.h>
#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint = 0, Input, Output = 0;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, 2,5,1, DIRECT);

PololuWheelEncoders encoders;

// encoder counts
double countsLeft;
double countsRight;
double speed = 75;
double sensorValues[5];
#define FRONT (0)
#define LEFT_FRONT (1)
#define LEFT_BACK (2)
#define RIGHT_FRONT (3)
#define RIGHT_BACK (4)

void setup() {
  // enables Serial communication
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
  myPID.SetMode(AUTOMATIC);

  analogWrite(PWMA, speed);    // uses PWM to set motor speed
  analogWrite(PWMB, speed);

  Serial.println ("SETUP COMPELETE");
  delay(5000);
}

void loop() {
  
  go(FORWARD,5*SQUARES);
  go(BACKWARD,5*SQUARES);

	// initialise maze
	
	// algorithm logic

	// get sensors function -- read the sensors and convert to cm
  // this should update the sensorValues array 
	getSensors();


	// check wall -- use sensor data to populate data for the node

	// get encoders -- encoder counts
}



//Give a direction (FORWARD, BACKWARD, LEFT, or RIGHT)
//and the number of encoder counts to move.
void go(int direction, int counts) {

  stopRobot();
  encoders.getCountsAndResetM1();
  encoders.getCountsAndResetM2();
  countsLeft = 0;
  countsRight = 0;

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

  if (direction == FORWARD || direction == BACKWARD) {

    while(abs(countsLeft) < counts || abs(countsRight) < counts) {
      
      Input = sensorValues[1] - sensorValues[3];
      myPID.Compute();
      analogWrite(PWMA, Output);

      countsLeft = encoders.getCountsM1();
      countsRight = encoders.getCountsM2();

      getSensors();
    }

  }

  else {

    while(abs(countsLeft) < counts || abs(countsRight) < counts) {
      countsLeft = encoders.getCountsM1();
      countsRight = encoders.getCountsM2();
    }

  }


  stopRobot();
}


// stop both motors and bring robot to a stop
void stopRobot() {
  digitalWrite(AIN1, LOW);  // stop left wheel
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);  // stop right wheel
  digitalWrite(BIN2, LOW);  
}



void getSensors() {

  sensorValues[LEFT_BACK] = FmultiMap(analogRead(A4), in, out, 18);
  sensorValues[LEFT_FRONT] = FmultiMap(analogRead(A3), in, out, 18);
  sensorValues[FRONT] = FmultiMap(analogRead(A2), in, out, 18);
  sensorValues[RIGHT_FRONT] = FmultiMap(analogRead(A1), in, out, 18);
  sensorValues[RIGHT_BACK] = FmultiMap(analogRead(A0), in, out, 18);

}