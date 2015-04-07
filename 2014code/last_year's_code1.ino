
/* Yale Bulldog Bots
 Software for BROWN IEEE Robotics Olympiad
 Christopher Datsikas, Margaret Ott
 Francois Kassier, Bernardo Saravia, Henry Li, and Devansh Tandon 
 4-11-2014
 // Adafruit Shield http://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/using-dc-motors
 // encoder fuctions http://www.pololu.com/docs/0J18/18
 IR sensor http://www.digikey.com/product-detail/en/GP2Y0A51SK0F/425-2854-ND/4103863
 Wheels http://www.pololu.com/product/1127

 A0- left
 A2- Right
 for 75 speed forward counts: 1470
 3.1 cm wheel diameter
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


int out[] = {
  150, 140, 130, 120, 110, 100, 90, 80, 70, 60, 50, 40, 35, 30, 25, 20, 15, 10};
// in[] holds the measured analogRead() values for defined distances
int in[]  = {
  86,95,100,108,120,130,145,161,180,203,232,272,301,329,364,400,470,522};
  /*
//Calibrated distance sensor - SHARP 2D120X F 24
// out[] holds the values wanted in mm
//int out[] = {
//  367,347, 182, 305, 287,267,252,237,225,213,201,191,177,167,157,142,132,122,112,102,92,82,72,62,57,52,47,42,37,34,29};
// in[] holds the measured analogRead() values for defined distances
//int in[]  = {
//  56,64,68,72,76,84,92,99,104,111,120,127,135,148,152,171,183,203,220,243,262,293,337,390,419,450,490,539,572,622,627};
*/
  //-----------------------------------------------------------


#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <PololuWheelEncoders.h>
#include <PID_v1.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(2); //left
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(1); //right

PololuWheelEncoders encoders;

int enCountsL=0; 
int enCountsR=0; 
double sp=75;
double input, output=0, setpoint=0;
long previousMillis = 0;        // will store last time LED was updated
byte counts=0;

PID myPID(&input, &output, &setpoint,.15,.10,.05, DIRECT);
boolean pidSwitch=AUTOMATIC;

boolean frontWall; // if there is a wall there
boolean rightWall;
boolean leftWall;

void setup() {
  Serial.begin(9600);  //starts up Serial communication
  AFMS.begin();       // starts up Motorsheild
  myMotor1->run(FORWARD);   //Starts motors with initial direction
  myMotor2->run(FORWARD);
  encoders_init(5,4,6,7);  //change to digital pin values
  myPID.SetMode(pidSwitch); //turns PID on
  myPID.SetSampleTime(100);
  myPID.SetOutputLimits(-255,255);
  delay(5000);
}

void loop(){
  getEncoders();
//  forwardOneSquare(sp);
 // getEncoders();
//  getSensors();
 // turnRight();
  delay(2000);
 // Serial.println("    ");
 // getEncoders();
 // getEncoders();
 // forwardAfterTurn(sp);
 // getEncoders();
 // Serial.println("Delay 3000 ms");
 //delay(3000);
 // Serial.println("--END OF LOOP--");

}

void forwardOneSquare(byte sp) {
  forward(sp, 1285);
}

void forwardAfterTurn(byte sp) {
  forward(sp, 535);
}

void forward(byte sp, int cnts){
  enCountsL=encoders.getCountsAndResetM1();
  enCountsR=encoders.getCountsAndResetM2();
  Serial.println("Forward One Square Function Start");
  Serial.print("EncoderL Count: ");
  Serial.println(enCountsL);
  Serial.print("EncoderR Counts: ");
  Serial.println(enCountsR);
  Serial.println(" ");
  myPID.SetMode(pidSwitch); //turns PID on
  myMotor1->run(FORWARD);   
  myMotor2->run(FORWARD);
  myMotor1->setSpeed(sp);
  myMotor2->setSpeed(sp);
  int diff=0; //set difference to 0
  boolean wallClose=false;
  while ( (enCountsL+enCountsR)/2 <cnts && !wallClose) {  //1487
    enCountsL=encoders.getCountsM1();
    enCountsR=encoders.getCountsM2();
    unsigned long currentMillis = millis();
    if(currentMillis - previousMillis > 20) {
      previousMillis = currentMillis; // save the last time you blinked the LED
      delay(1);
      int x1= analogRead(A0);
      int x2= analogRead(A2);
      int y1= FmultiMap(x1, in, out, 18);
      int y2= FmultiMap(x2, in, out, 18);
      Serial.print(x1);
      Serial.print("  ");
      Serial.println(x2);
      Serial.print(y1);
      Serial.print("  ");
      Serial.println(y2);
      int z=y1-y2;
      Serial.print("z: ");
      Serial.println(z);
      diff= diff+ z;
      wallClose=checkWall();
      counts=counts+1;
      if (counts==3) {
        input=diff/3;
        diff=0;
        counts=0;
      }
    }
    if (abs(input) >16) myPID.SetMode(MANUAL);
    else myPID.SetMode(pidSwitch);

   Serial.print("Input : ");
    Serial.println(input);
    myPID.Compute(); 
    Serial.print("Output:   ");
    Serial.println(output);
    Serial.print("Speed:  ");
    Serial.println(constrain(sp+output,0,255));
    Serial.println( " ");
    myMotor1->setSpeed(constrain(sp-output,0,255));
    myMotor2->setSpeed(constrain(sp+output,0,255));
  }   
  myMotor1->setSpeed(0);
  myMotor2->setSpeed(0);
  Serial.println("Forward One Square Function End");
  Serial.print("EncoderL Count: ");
  Serial.println(enCountsL);
  Serial.print("EncoderR Counts: ");
  Serial.println(enCountsR);
  Serial.println(" ");
  enCountsL=encoders.getCountsAndResetM1();
  enCountsR=encoders.getCountsAndResetM2();
}

void backwardOneSquare(byte sp){
  enCountsL=encoders.getCountsAndResetM1();
  enCountsR=encoders.getCountsAndResetM2();
  myMotor1->run(BACKWARD);   
  myMotor2->run(BACKWARD);
  myMotor1->setSpeed(sp);
  myMotor2->setSpeed(sp);
  while (enCountsL >-1285) {
    enCountsL=encoders.getCountsM1();
    enCountsR=encoders.getCountsM2();
  }
  myMotor1->setSpeed(0);
  myMotor2->setSpeed(0);
  Serial.println("Backward One Square Function");
  Serial.print("EncoderL Count: ");
  Serial.println(enCountsL);
  Serial.print("EncoderR Counts: ");
  Serial.println(enCountsR);
  Serial.println(" ");
  enCountsL=encoders.getCountsAndResetM1();
  enCountsR=encoders.getCountsAndResetM2();
}


void turnRight() {
  myPID.SetMode(MANUAL); //turns PID off
  enCountsL=encoders.getCountsAndResetM1();
  enCountsR=encoders.getCountsAndResetM2();
  myMotor1->run(FORWARD);   
  myMotor2->run(FORWARD);
  myMotor1->setSpeed(100);
  myMotor2->setSpeed(0);
  Serial.println("Turn Right Function Start");
  Serial.print("EncoderL Count: ");
  Serial.println(enCountsL);
  Serial.print("EncoderR Counts: ");
  Serial.println(enCountsR);
  Serial.println(" ");
  while (enCountsL <1000) { //1300
    enCountsL=encoders.getCountsM1();
    enCountsR=encoders.getCountsM2();
  }
  myMotor1->setSpeed(0);
  myMotor2->setSpeed(0);
  myPID.Compute(); 
  Serial.print("Output:   ");
  Serial.println(output);
  Serial.print("Speed:  ");
  Serial.println(constrain(sp+output,0,255));
  Serial.println("Turn Right Function End");
  Serial.print("EncoderL Count: ");
  Serial.println(enCountsL);
  Serial.print("EncoderR Counts: ");
  Serial.println(enCountsR);
  Serial.println(" ");
  enCountsL=encoders.getCountsAndResetM1();
  enCountsR=encoders.getCountsAndResetM2();
}

void turnLeft() {
  myPID.SetMode(MANUAL); //turns PID off
  enCountsL=encoders.getCountsAndResetM1();
  enCountsR=encoders.getCountsAndResetM2();
  myMotor1->run(FORWARD);   
  myMotor2->run(FORWARD);
  myMotor1->setSpeed(0);
  myMotor2->setSpeed(100);
  while (enCountsR <1000) {  //1300
    enCountsL=encoders.getCountsM1();
    enCountsR=encoders.getCountsM2();
  }
  myMotor1->setSpeed(0);
  myMotor2->setSpeed(0);
  enCountsL=encoders.getCountsAndResetM1();
  enCountsR=encoders.getCountsAndResetM2();
}

void getSensors() {
  int sensorF_a=analogRead(A1);
  int sensorL1_a=analogRead(A2);
  int sensorL2_a=analogRead(A3);
  int sensorR1_a=analogRead(A0);
  int sensorR2_a=analogRead(A5);
  int test=analogRead(A4);
  delay(20);
  int sensorF_b=analogRead(A1);
  int sensorL1_b=analogRead(A2);
  int sensorL2_b=analogRead(A3);
  int sensorR1_b=analogRead(A0);
  int sensorR2_b=analogRead(A5);
  delay(20);
  int sensorF_c=analogRead(A1);
  int sensorL1_c=analogRead(A2);
  int sensorL2_c=analogRead(A3);
  int sensorR1_c=analogRead(A0);
  int sensorR2_c=analogRead(A5);

  int mmF = FmultiMap((sensorF_a+sensorF_b+sensorF_c)/3, in, out, 18);
  int mmL1 = FmultiMap((sensorL1_a+sensorL1_b+sensorL1_c)/3, in, out, 18);
  int mmL2 = FmultiMap((sensorL2_a+sensorL2_b+sensorL2_c)/3, in, out, 18);
  int mmR1 = FmultiMap((sensorR1_a+sensorR1_b+sensorR1_c)/3, in, out, 18);
  int mmR2 = FmultiMap((sensorR2_a+sensorR2_b+sensorR2_c)/3, in, out, 18);

  if (mmF <100) {
    frontWall= true;
  }
  else {
    frontWall=false;
  }

  if (mmL1 <100) {
    leftWall= true;
  }
  else {
    leftWall=false;
  }

  if (mmR1 <100) {
    rightWall=true;
  }
  else {
    rightWall=false;
  }


  Serial.println("Get Sensors Function");
  Serial.print("Sensor F Value: ");
  Serial.print(sensorF_a);
  Serial.print("  ");
  Serial.print(sensorF_b);
  Serial.print("  ");
  Serial.print(sensorF_c);
  Serial.print("   ");
  Serial.println(mmF);  
  Serial.print("Sensor L1 Value: ");
  Serial.print(sensorL1_a);
  Serial.print("   ");
  Serial.print(sensorL1_b);
  Serial.print("   ");
  Serial.print(sensorL1_c);
  Serial.print("   ");
  Serial.println(mmL1);
  Serial.print("Sensor L2 Value: ");
  Serial.print(sensorL2_a);
  Serial.print("   ");
  Serial.print(sensorL2_b);
  Serial.print("   ");
  Serial.print(sensorL2_c);
  Serial.print("   ");
  Serial.println(mmL2);
  Serial.print("Sensor R1 Value: ");
  Serial.print(sensorR1_a);
  Serial.print("  ");
  Serial.print(sensorR1_b);
  Serial.print("  ");
  Serial.print(sensorR1_c);
  Serial.print("  ");
  Serial.println(mmR1);
  Serial.print("Sensor R2 Value: ");
  Serial.print(sensorR2_a);
  Serial.print("  ");
  Serial.print(sensorR2_b);
  Serial.print("  ");
  Serial.print(sensorR2_c);
  Serial.print("  ");
  Serial.println(mmR2);
  Serial.print("Sensor test Value: ");
  Serial.println(test);   
  Serial.println(" ");
}

void getEncoders(){
  enCountsL=encoders.getCountsM1();
  enCountsR=encoders.getCountsM2();
  Serial.println("Get Encoders Function");
  Serial.print("EncoderL Counts: ");
  Serial.println(enCountsL);
  Serial.print("EncoderR Counts: ");
  Serial.println(enCountsR);
  Serial.println(" ");
}

boolean checkWall() {
  int frontSensor=FmultiMap(analogRead(A1), in, out, 18);
  if(frontSensor < 37) {
    return true;
  }
  else {
    return false;
  }
}




