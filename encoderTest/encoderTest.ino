/* encoderTest.ino
 Christopher Datsikas
 Created: Mar 2015
 Last Modified: 04-04-2015 by Richard
 Status: Works
 
 Tests the Pololu Encoders
 connected to Arduino Nano
 https://www.pololu.com/product/2598
 
 Requires Pololu Wheel Encoders Library for Arduino
 http://www.pololu.com/file/download/PololuArduinoLibraries-111221.zip?file_id=0J521
 Download entire library as zip
 Copy entire src folder into libraries folder in sketchbook
 
 Connections:
 Ard --  Encoder
 D9  --  M1OUTA  
 D10 --  M1OUTB  
 D11 --  M2OUTB  
 D12 --  M2OUTA
 5V  --  5V
 GND --  GND

 Encoder counts increment when wheel spins forward,
                decrement when wheel spins backwards.

  ~910 encoder counts per revolution.
 */

 
#include <PololuWheelEncoders.h>


void setup() {
  Serial.begin(9600);  
  PololuWheelEncoders::init(10,9,12,11);
}


void loop() {
  Serial.print(PololuWheelEncoders::getCountsM1());
  Serial.print("   ");
  Serial.println(PololuWheelEncoders::getCountsM2());
  delay(200);
}



//void getEncoders(){
//  enCountsL=encoders.getCountsM1();
//  enCountsR=encoders.getCountsM2();
//  Serial.println("Get Encoders Function");
//  Serial.print("EncoderL Count: ");
//  Serial.println(enCountsL);
//  Serial.print("EncoderR Counts: ");
//  Serial.println(enCountsR);
//  Serial.println(" ");
//}

