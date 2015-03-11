/*
TB6612FNG-TEST.ino


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
  Serial.begin(9600);
  // set all the Motor Driver pins as OUTPUT
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(AIN1, HIGH); //turns motor clockwise
  digitalWrite(AIN2, LOW);
  digitalWrite(PWMA, 2);
  delay(5000);
  digitalWrite(AIN1, LOW);  // turn motor counter-clockwise
  digitalWrite(AIN2, HIGH);
  digitalWrite(PWMA, 200);
  delay(5000);
}
