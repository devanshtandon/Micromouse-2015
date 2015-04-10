/* MotorEncoderTest.ino
Richard Chang
Created: April 2015
Last Modified: 04-04-2015
Status: Works, but needs more fine tuning

Test the motor-encoder combination.

Use the go(direction, distance) function to tell the robot
which direction and how far (in encoder counts) to move.

See CONSTANTS for the number of encoder counts per square (16.8cm)
or 90 degree turn. NEEDS FINE TUNING.

 */

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
const int RIGHT_TURN=630;
const int LEFT_TURN=755;


#include <PololuWheelEncoders.h>
PololuWheelEncoders encoders;

int countsM1;
int countsM2;


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

  analogWrite(PWMA, 100);    // left wheel 
  analogWrite(PWMB, 100);  // right wheel
}


void loop() {

 go(FORWARD,1*SQUARE);
// delay(500);
 go(RIGHT,1*RIGHT_TURN);
// delay(500);

  // Serial.print(PololuWheelEncoders::getCountsM1());
  // Serial.print("   ");
  // Serial.println(PololuWheelEncoders::getCountsM2());
  // delay(200);

  // go(FORWARD,4*SQUARES);
  // delay(1000);
  // go(BACKWARD,4*SQUARES);
  // delay(1000);

  // go(FORWARD,1*SQUARE);
  // go(LEFT,1*TURN);
  // go(FORWARD,1*SQUARE);
  // go(RIGHT,1*TURN);
  // go(FORWARD,1*SQUARE);
  // go(RIGHT,1*TURN);
  // go(FORWARD,1*SQUARE);
  // go(RIGHT,1*TURN);
  // go(FORWARD,2*SQUARES);
  // go(LEFT,2*TURNS);
  // delay(1000);
}




//Give a direction (FORWARD, BACKWARD, LEFT, or RIGHT)
//and a distance (in number of encoder counts) to move.
void go(int direction, int distance) {

  PORTD &= B01110011; // stop first
  PORTB &= 255-(1<<0); 
  encoders.getCountsAndResetM1();
  encoders.getCountsAndResetM2();
  countsM1 = 0;
  countsM2 = 0;

  if (direction == FORWARD) {
    PORTD &= 255-(1<<3); 
    PORTD |= B10000100;
    PORTB &= 255-(1<<0); 
  }

  else if (direction == BACKWARD) {
    PORTD |= (1<<3);  
    PORTD &= B01111011;
    PORTB |= (1<<0);  
  }

  else if (direction == LEFT) {
    PORTD |= B10001000;
    PORTD &= 255-(1<<2);
    PORTB &= 255-(1<<0);
  }

  else if (direction == RIGHT) {
    PORTD &= B01110111;
    PORTD |= (1<<2);
    PORTB |= (1<<0); 
  }

  while(abs(countsM1) < distance || abs(countsM2) < distance) {
    countsM1 = encoders.getCountsM1();
    countsM2 = encoders.getCountsM2();
  }

  PORTD &= B01110011; // stop
  PORTB &= 255-(1<<0); 

}



// stop both motors and bring robot to a stop
void stopRobot() {
  PORTD &= B01110011; 
  PORTB &= 255-(1<<0);
}
