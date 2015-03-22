/* micromouse2015.ino 
Christopher Datsikas

Created: 03-16-2015
Last Modified: 03-22-2015
Status: In Progress
Arduino Code for Micromouse for 
2015 Brown IEEE Robotics Competition

Github: https://github.com/devanshtandon/Micromouse-2015
Upverter: https://upverter.com/chrisdats/26efdb039ba46d67/Micromouse2015/

Components:
- TB6612FNG Dual Motor Driver Carrier - https://www.pololu.com/product/713
- 75:1 Micro Metal Gearmotor HP with Extended Motor Shaft - https://www.pololu.com/product/2215
- Sharp GP2Y0A51SK0F Analog Distance Sensor 2-15cm - https://www.pololu.com/product/2450
- Magnetic Encoders 12 CPR https://www.pololu.com/product/2598

Connections:
ARD -- OTHER COMPONEENTS
D2  -- AIN2 Motor Driver
D3  -- AIN1 Motor Driver
D4  -- STBY Motor Driver
D5 (pwm) -- PWMA Motor Driver
D6 (pwm) -- PWMB Motor Driver
D7  -- BIN1 Motor Driver
D8  -- BIN2 Motor Driver

D9  -- OUTA Encoder 1
D10 -- OUTB Encoder 1

D11 -- OUTB Encoder 2 // this looks switched
D12 -- OUTA Encoder 2 // done on purpose; to make pcb easier

A0  -- J5 Sharp Distance Sensor (rightmost)
A1  -- J4 Sharp Distance Sensor
A2  -- J3 Sharp Distance Sensor
A3  -- J2 Sharp Distance Sensor
A4  -- J1 Sharp Distance Sensor (leftmost

GND -- GND // make sure all grounds are connected

MOTOR DRIVER -- ENCODERS
AO1 -- M1 Encoder 1
AO2 -- M2 Encoder 1

BO1 -- M1 Encoder 2
BO2 -- M2 Encoder 2

*/

// define pins for Motor Driver

void setup() {
  // enables Serial communication
  Serial.begin(9600);

  // set motor driver pins to OUTPUT

  Serial.println ("SETUP COMPELETE");
  delay(5000);
}

void loop() {
// fill
}
