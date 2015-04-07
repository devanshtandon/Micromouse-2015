/* RangeFinderTest.ino
 Richard Chang
 Created: April 2015
 Last Modified: 04-07-2015
 Status: Kind of works, but needs more calibration
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


// Left back sensor (J1) -- A4
// Left forward sensor (J2) -- A3
// Front sensor (J3) -- A2
// Right forward sensor (J4) -- A1
// Right back sensor (J5) -- A0



#include <Wire.h>
#include <PololuWheelEncoders.h>
#include <PID_v1.h>


PololuWheelEncoders encoders;

const float WHEEL_DIAMETER=3.12; // 3.12 cm, 31.2 mm
const float WHEEL_CIRCUM=3.141592*WHEEL_DIAMETER; //C=PI*D
const float WIDTH=2; //distance between center of two wheels


void setup() {
  Serial.begin(9600);  //starts up Serial communication
  delay(2000);
}

void loop(){

  int sensorLb=analogRead(A4);
  int sensorLf=analogRead(A3);
  int sensorF=analogRead(A2);
  int sensorRf=analogRead(A1);
  int sensorRb=analogRead(A0);

  int mm1 = FmultiMap(sensorLb, in, out, 18);
  int mm2 = FmultiMap(sensorLf, in, out, 18);
  int mm3 = FmultiMap(sensorF, in, out, 18);
  int mm4 = FmultiMap(sensorRf, in, out, 18);
  int mm5 = FmultiMap(sensorRb, in, out, 18);

  Serial.print("Left back sensor value: ");
  Serial.print(sensorLb);
  Serial.print("   ");
  Serial.println(mm1);
  Serial.print("Left forward sensor value: ");
  Serial.print(sensorLf);
  Serial.print("   ");
  Serial.println(mm2);
  Serial.print("Front sensor value: ");
  Serial.print(sensorF);
  Serial.print("   ");
  Serial.println(mm3);
  Serial.print("Right forward sensor value: ");
  Serial.print(sensorRf);
  Serial.print("   ");
  Serial.println(mm4);
  Serial.print("Right back sensor value: ");
  Serial.print(sensorRb);
  Serial.print("   ");
  Serial.println(mm5);
  Serial.println(" ");

  Serial.println(" ");
  delay(3000);

}



