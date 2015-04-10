/* motorTest.ino
Richard Chang
Created: April 2015
Last Modified: 04-04-2015
Status: Works

Test the Pololu TB6612FNG Dual Motor Driver Carrier
connected to an Arduino Nano and Pololu Micro Metal Gearmotor
// http://www.embeddedrelated.com/showarticle/498.php

Test the go(direction) function.
Robot will move FORWARD, BACKWARD, LEFT, RIGHT, STOP, and then repeat
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


const float WHEEL_DIAMETER=3.12; // 3.12 cm, 31.2 mm
const float WHEEL_CIRCUM=3.141592*WHEEL_DIAMETER; //C=PI*D ~9.8018 cm
const float COUNTS_PER_REV=909.72; // 75.81 x 12
const float COUNTS_PER_CM=COUNTS_PER_REV/WHEEL_CIRCUM; // ~92.81
const int INT_COUNTS_PER_CM=93; // use int for faster arithmetic
const float WIDTH=2; //distance between center of two wheels

int direction = 0;


void setup() {
  // enables serial communication
  Serial.begin(9600);  

  // set all the Motor Driver pins to OUTPUT
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);  // turns motor driver on

  analogWrite(PWMA, 100);    // uses PWM to set motor speed
  analogWrite(PWMB, 100);

}

void loop() {
  direction = (direction + 1)%5;
  go(direction);
  delay(2000); 
  go(STOP);
  delay(1000);
}





//give a direction (STOP, FORWARD, BACKWARD, LEFT, or RIGHT)
//register commands are used for speed (see below for comments)
void go(int direction) {

  if (direction == STOP) {
    PORTD &= B01110011; 
    PORTB &= 255-(1<<0);
  }

  if (direction == FORWARD) { 
    PORTD &= 255-(1<<3); 
    PORTD |= B10000100;
    PORTB &= 255-(1<<0); 
  }

  if (direction == BACKWARD) {
    PORTD |= (1<<3);  
    PORTD &= B01111011;
    PORTB |= (1<<0);  
  }

  if (direction == LEFT) {
    PORTD |= B10001000;
    PORTD &= 255-(1<<2);
    PORTB &= 255-(1<<0);
  }

  if (direction == RIGHT) {
    PORTD &= B01110111;
    PORTD |= (1<<2);
    PORTB |= (1<<0); 
  }

}



// void go(int direction) {

//   if (direction == STOP) {
//     PORTD &= 255-(1<<3); //digitalWrite(AIN1, LOW);  
//     PORTD &= 255-(1<<2); //digitalWrite(AIN2, LOW);
//     PORTD &= 255-(1<<7); //digitalWrite(BIN1, LOW);  
//     PORTB &= 255-(1<<0); //digitalWrite(BIN2, LOW);  
//   }

//   if (direction == FORWARD) { 
//     PORTD &= 255-(1<<3); //digitalWrite(AIN1, LOW);   // Left wheel forward
//     PORTD |= (1<<2);     //digitalWrite(AIN2, HIGH);
//     PORTD |= (1<<7);     //digitalWrite(BIN1, HIGH);  // Right wheel forward
//     PORTB &= 255-(1<<0); //digitalWrite(BIN2, LOW);  
//   }

//   if (direction == BACKWARD) {
//     PORTD |= (1<<3);     //digitalWrite(AIN1, HIGH); // Left wheel reverse
//     PORTD &= 255-(1<<2); //digitalWrite(AIN2, LOW);
//     PORTD &= 255-(1<<7); //digitalWrite(BIN1, LOW);  // Right wheel reverse
//     PORTB |= (1<<0);     //digitalWrite(BIN2, HIGH);
//   }

//   if (direction == LEFT) {
//     PORTD |= (1<<3);     //digitalWrite(AIN1, HIGH);  // Left wheel reverse
//     PORTD &= 255-(1<<2); //digitalWrite(AIN2, LOW);
//     PORTD |= (1<<7);     //digitalWrite(BIN1, HIGH);  // Right wheel forward
//     PORTB &= 255-(1<<0); //digitalWrite(BIN2, LOW);
//   }

//   if (direction == RIGHT) {
//     PORTD &= 255-(1<<3); //digitalWrite(AIN1, LOW);  // Left wheel forward
//     PORTD |= (1<<2);     //digitalWrite(AIN2, HIGH);
//     PORTD &= 255-(1<<7); //digitalWrite(BIN1, LOW);  // Right wheel reverse
//     PORTB |= (1<<0);     //digitalWrite(BIN2, HIGH);
//   }

// }



// stop both motors and bring robot to a stop
void stopRobot() {
  PORTD &= B01110011; 
  PORTB &= 255-(1<<0);
}
