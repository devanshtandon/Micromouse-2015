/* Yale Bulldog Bots
 Software for BROWN IEEE Robotics Olympiad
 Christopher Datsikas, Margaret Ott
 Francois Kassier, Bernardo Saravia, Henry Li, and Devansh Tandon 
 3-22-2014
 // Adafruit Shield http://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/using-dc-motors
 // encoder fuctions http://www.pololu.com/docs/0J18/18
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

//Calibrated distance sensor - SHARP 2D120X F 24
// out[] holds the values wanted in mm
int out[] = {
  367,347, 312, 305, 287,267,252,237,225,213,201,191,177,167,157,142,132,122,112,102,92,82,72,62,57,52,47,42,37,34,29};
// in[] holds the measured analogRead() values for defined distances
int in[]  = {
  56,64,68,72,76,84,92,99,104,111,120,127,135,148,152,171,183,203,220,243,262,293,337,390,419,450,490,539,572,622,627};
//-----------------------------------------------------------


#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <PololuWheelEncoders.h>
#include <PID_v1.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);

PololuWheelEncoders encoders;

const float WHEEL_DIAMETER=3.12; // 3.12 cm, 31.2 mm
const float WHEEL_CIRCUM=3.141592*WHEEL_DIAMETER; //C=PI*D
const float WIDTH=2; //distance between center of two wheels


void setup() {
  Serial.begin(9600);  //starts up Serial communication
  delay(2000);
}

void loop(){

  int sensorF=analogRead(A1);
  int sensorL1=analogRead(A2);
  int sensorL2=analogRead(A3);
  int sensorR1=analogRead(A0);
  int test=analogRead(A4);
  int sensorR2=analogRead(A5);
  int mm1 = FmultiMap(sensorF, in, out, 31);
  int mm2 = FmultiMap(sensorL1, in, out, 31);
  int mm3 = FmultiMap(sensorL2, in, out, 31);
  int mm4 = FmultiMap(sensorR1, in, out, 31);
  int mm5 = FmultiMap(sensorR2, in, out, 31);
  int mmtest = FmultiMap(test, in, out, 31);

  Serial.print("Sensor F Value: ");
  Serial.print(sensorF);
  Serial.print("   ");
  Serial.println(mm1);
  Serial.print("Sensor L1 Value: ");
  Serial.print(sensorL1);
  Serial.print("   ");
  Serial.println(mm2);
  Serial.print("Sensor L2 Value: ");
  Serial.print(sensorL2);
  Serial.print("   ");
  Serial.println(mm3);
  Serial.print("Sensor R1 Value: ");
  Serial.print(sensorR1);
  Serial.print("   ");
  Serial.println(mm4);
  Serial.print("Sensor R2 Value: ");
  Serial.print(sensorR2);
  Serial.print("   ");
  Serial.println(mm5);
  Serial.print("Sensor test Value: ");
  Serial.print(test);
  Serial.print("   ");
  Serial.print(mmtest);  
  Serial.println(" ");

  Serial.println(" ");
  delay(3000);

}



