/* encoderTest.ino
 Christopher Datsikas
 Created: Mar 2015
 Last Modified: 03-22-2015
 Status: Works
 
 Tests the Pololu Encoders
 connected to Arduino Nano
 https://www.pololu.com/product/2598
 
 Requires Pololu Wheel Encoders Library
 https://github.com/pololu/libpololu-avr 
 Download entire library as zip
 Copy entire src folder into libraries folder in sketchbook
 
 Connections:
 Ard --  Encoder
 D9  --  M1OUTA  (looks wrong)
 D10 --  M1OUTB  (but this is know the PCB is setup to make routing easier)
 D11 --  M2OUTB  (change arguments to encoders_init accordingly)
 D12 --  M2OUTA
 5V  --  5V
 GND --  GND
 
 */

#include <PololuWheelEncoders.h>
PololuWheelEncoders encoders;

int countsM1;
int countsM2;

void setup() {
  // enables serial communication
  Serial.begin(9600);  

  // initializes the encoders; no pinMode calls are necessary  
  encoders_init(10,9,11,12); //9-12
}

void loop() {
  countsM1 = encoders.getCountsM1();
  countsM2 = encoders.getCountsM2();
  Serial.print(countsM1);
  Serial.print("   ");
  Serial.println(countsM2);
  delay(200);
}


