#include <PololuWheelEncoders.h>
PololuWheelEncoders encoders;

int enCountsA;
int enCountsB;

void setup() {
encoders_init(8,9,255,255); 
Serial.begin(9600);
}

void loop() {
  enCountsA=encoders.getCountsM1();
   Serial.println(encoderAcounts);
}

