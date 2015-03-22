/* TB6612FNG-TEST.ino
Christopher Datsikas
Created: Feb 2015
Last Modified: 03-11-2015
Status: Works

Test the Pololu TB6612FNG Dual Motor Driver Carrier
connected to an Arduino Nano and Pololu Micro Metal Gearmotor
// http://www.embeddedrelated.com/showarticle/498.php
 */

// Define pins for Motor Driver TB6612FNG
#define PWMA 5
#define PWMB 6

#define AIN2 2
#define AIN1 3
#define BIN1 7
#define BIN2 8

#define STBY 4

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
}

void loop() {
  // turns motor clockwise
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 100);    // uses PWM to set motor speed
  delay(5000);            // wait for five seconds
  
   // turn motor counter-clockwise
  digitalWrite(AIN1, LOW); 
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, 200);   // uses PWM to set motor speed
  delay(5000);              // wait for five seconds
}
