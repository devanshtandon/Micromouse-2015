
/* Yale Bulldog Bots
 Software for BROWN IEEE Robotics Olympiad
 Christopher Datsikas, Margaret Ott
 Francois Kassier, Bernardo Saravia, Henry Li, and Devansh Tandon 
 4-12-2014
 // Adafruit Shield http://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/using-dc-motors
 // encoder fuctions http://www.pololu.com/docs/0J18/18
 IR sensor http://www.digikey.com/product-detail/en/GP2Y0A51SK0F/425-2854-ND/4103863
 Wheels http://www.pololu.com/product/1127

 A0- left
 A2- Right
 
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

//Calibrated distance sensor - SHARP 2D120X F 24
// out[] holds the values wanted in mm
int out[] = {
  150,140, 130, 120, 110, 100, 90, 80, 70, 60, 50, 40, 35, 30, 25, 20, 15, 10};
// in[] holds the measured analogRead() values for defined distances
int in[]  = {
  86,95,100,108,120,130,145,161,180,203,232,272,301,329,364,400,470,522};
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

#define NORTH      1
#define EAST       2
#define SOUTH      4
#define WEST       8

// maze storage
#define NUMCELLS 256
unsigned char maze[NUMCELLS]; // array holding wall info for each cell
unsigned char mapx[NUMCELLS]; // array holding distance value of each cell
unsigned char possibleMoves[4]; // array holding possible moves from given square

unsigned char i, j = 0; // i is what cell robot is in (from 0 to 255)

unsigned char state = NORTH;
byte goal=118;

boolean frontWall;
boolean rightWall;
boolean leftWall;

boolean northWall;
boolean eastWall;
boolean westWall;
boolean southWall;

void setup() {
  Serial.begin(9600);  //starts up Serial communication
  AFMS.begin();       // starts up Motorsheild
  myMotor1->run(FORWARD);   //Starts motors with initial direction
  myMotor2->run(FORWARD);
  encoders_init(5,4,6,7);  //change to digital pin values
  myPID.SetMode(pidSwitch); //turns PID on
  myPID.SetSampleTime(100);
  myPID.SetOutputLimits(-255,255);
  
   //fill maze and mapx with 0's
  for (int k = 0; k < NUMCELLS; k++) {
    maze[k] = 0;
    mapx[k] = 0;
  }

  maze[0] += SOUTH;
  
  Serial.println ("SETUP COMPELETE");
  delay(5000);
}

void loop(){
if ((i == 118) || (i == 120) || (i == 135) || (i == 136)) {
    // stop, you are at the middle
    // client.stop();
  }

  // sense things and get four booleans
  getSensors();
  
  // change maze array based on what walls are there
  if (northWall) {
    Serial.print("maze[i]: ");
    Serial.println(maze[i]);
    if ((maze[i] & NORTH) == 0) {
      maze[i] += NORTH;
    }
    
    j = i + 1;
    if ((maze[j] & SOUTH) == 0) {
      maze[j] += SOUTH;
    }
    
  }
  
  if (eastWall) {
    
    if ((maze[i] & EAST) == 0) {
      maze[i] += EAST;
    }
    
    j = i + 16;
    if ((maze[j] & WEST) == 0) {
      maze[j] += WEST;
    }
    
  }
  
  if (westWall) {
    
    if ((maze[i] & WEST) == 0) {
      maze[i] += WEST;
    }
    
    j = i + 240;
    if ((maze[j] & EAST) == 0) {
      maze[j] += EAST;
    }
    
  }
  
  if (southWall) {
    
    if ((maze[i] & SOUTH) == 0) {
      maze[i] += SOUTH;
    }
    
    j = i + 255;
    if ((maze[j] & NORTH) == 0) {
      maze[j] += NORTH;
    }
  }
  
  floodMaze(goal);
  
  /* sort four cell's distance values (not separated by walls)
  and move to the smallest one */
  int cellwalls = maze[i];
  if (state == NORTH) {
    
    if ((cellwalls & NORTH) == 0) { //if there is no wall to the North
      possibleMoves[0] = mapx[i+1];
    } else {
      possibleMoves[0] = 255;
    }
    
    if ((cellwalls & EAST) == 0) {
      possibleMoves[1] = mapx[i+16];
    } else {
      possibleMoves[1] = 255;
    }
    
    if ((cellwalls & SOUTH) == 0) {
      possibleMoves[2] = mapx[i+255];
    } else {
      possibleMoves[2] = 255; 
    }
    
    if ((cellwalls & WEST) == 0) {
      possibleMoves[3] = mapx[i+240];
    } else {
      possibleMoves[3] = 255;
    }
    
  } else if (state == SOUTH) {
    
    if ((cellwalls & SOUTH) == 0) {
      possibleMoves[0] = mapx[i+255];
    } else {
      possibleMoves[0] = 255; 
    }
    
    if ((cellwalls & WEST) == 0) {
      possibleMoves[1] = mapx[i+240];
    } else {
      possibleMoves[1] = 255;
    }
    
    if ((cellwalls & NORTH) == 0) {
      possibleMoves[2] = mapx[i+1];
    } else {
      possibleMoves[2] = 255;
    }
    
    if ((cellwalls & EAST) == 0) {
      possibleMoves[3] = mapx[i+16];
    } else {
      possibleMoves[3] = 255;
    }
    
  } else if (state == EAST) {
    
    if ((cellwalls & EAST) == 0) {
      possibleMoves[0] = mapx[i+16];
    } else {
      possibleMoves[0] = 255;
    }
    
    if ((cellwalls & SOUTH) == 0) {
      possibleMoves[1] = mapx[i+255];
    } else {
      possibleMoves[1] = 255; 
    }
    
    if ((cellwalls & WEST) == 0) {
      possibleMoves[2] = mapx[i+240];
    } else {
      possibleMoves[2] = 255;
    }
    
    if ((cellwalls & NORTH) == 0) {
      possibleMoves[3] = mapx[i+1];
    } else {
      possibleMoves[3] = 255;
    }
  
  } else if (state == WEST) {
    
    if ((cellwalls & WEST) == 0) {
      possibleMoves[0] = mapx[i+240];
    } else {
      possibleMoves[0] = 255;
    }
    
    if ((cellwalls & NORTH) == 0) {
      possibleMoves[1] = mapx[i+1];
    } else {
      possibleMoves[1] = 255;
    }
    
    if ((cellwalls & EAST) == 0) {
      possibleMoves[2] = mapx[i+16];
    } else {
      possibleMoves[2] = 255;
    }
    
    if ((cellwalls & SOUTH) == 0) {
      possibleMoves[3] = mapx[i+255];
    } else {
      possibleMoves[3] = 255; 
    }
    
  }
  
  int smallest = findSmallest(possibleMoves);
  
  if (smallest == 0) {
    forwardOneSquare(sp);
  } else if (smallest == 1) {
    turnRight();
    forwardAfterTurn(sp);
  } else if (smallest == 2) {
    backwardOneSquare(sp);
  } else if (smallest == 3) {
    turnLeft();
    forwardAfterTurn(sp);
  }

/*
Serial.print("state: ");
Serial.println(state);
Serial.print( "maze" );

int r;
for (r = 0; r < 255; r = r + 1) {
  Serial.print(maze[r]);
  Serial.print(" ");
  if (r%16==0) {
  Serial.println(" ");
  }
}
Serial.println(" ");

Serial.print("mapx");
for (r = 0; r < 255; r = r + 1) {
  Serial.print(mapx[r]);
  Serial.print(" ");
  if (r%16==0) {
  Serial.println(" ");
  }
}

int e;
for (e=0; e<3; e=e+1) {
  Serial.print(possibleMoves[e]);
}

delay(10000);
*/

}
// ******************************* end loop 


void forwardOneSquare(byte sp) {
  forward(sp, 1285);
}

void forwardAfterTurn(byte sp) {
  forward(sp, 535);
}

void forward(byte sp, int cnts){
    if (state == NORTH) {
    i = i + 1;
  } else if (state == SOUTH) {
    i = i - 1;
  } else if (state == WEST) {
    i = i - 16;
  } else if (state == EAST) {
    i = i + 16;
  }
  
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
    if (state == NORTH) {
    i = i - 1;
  } else if (state == SOUTH) {
    i = i + 1;
  } else if (state == WEST) {
    i = i + 16;
  } else if (state == EAST) {
    i = i - 16;
  }
  
  enCountsL=encoders.getCountsAndResetM1();
  enCountsR=encoders.getCountsAndResetM2();
  myMotor1->run(BACKWARD);   
  myMotor2->run(BACKWARD);
  myMotor1->setSpeed(sp+3);
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
  Serial.println("TURN RIGHT");
    if (state == NORTH) {
    state -= NORTH;
    state += EAST;
  } else if (state == SOUTH) {
    state -= SOUTH;
    state += WEST;
  } else if (state == WEST) {
    state -= WEST;
    state += NORTH;
  } else if (state == EAST) {
    state -= EAST;
    state += SOUTH;
  }
  
  myPID.SetMode(MANUAL); //turns PID off
  enCountsL=encoders.getCountsAndResetM1();
  enCountsR=encoders.getCountsAndResetM2();
  myMotor1->run(FORWARD);   
  myMotor2->run(FORWARD);
  myMotor1->setSpeed(75);
  myMotor2->setSpeed(0);
  Serial.println("Turn Right Function Start");
  Serial.print("EncoderL Count: ");
  Serial.println(enCountsL);
  Serial.print("EncoderR Counts: ");
  Serial.println(enCountsR);
  Serial.println(" ");
  while (enCountsL <1000) {   //1300
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
   Serial.println ("TURN LEFT");
   
    if (state == NORTH) {
    state -= NORTH;
    state += WEST;
  } else if (state == SOUTH) {
    state -= SOUTH;
    state += EAST;
  } else if (state == WEST) {
    state -= WEST;
    state += SOUTH;
  } else if (state == EAST) {
    state -= EAST;
    state += NORTH;
  }
  
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
     Serial.println("frontWall= true;");
  }
  else {
    frontWall=false;
  }

  if (mmL1 <100) {
    leftWall= true;
    Serial.println("leftWall= true;");
  }
  else {
    leftWall=false;
  }

  if (mmR1 <100) {
    rightWall=true;
    Serial.println("rightWall= true;");
  }
  else {
    rightWall=false;
  }

if (state == NORTH) {
    northWall = frontWall;
    eastWall = rightWall;
    westWall = leftWall;
    southWall = false;
  } else if (state == SOUTH) {
    southWall = frontWall;
    eastWall = leftWall;
    westWall = rightWall;
    frontWall = false;
  } else if (state == WEST) {
    westWall = frontWall;
    northWall = rightWall;
    southWall = leftWall;
    eastWall = false;
  } else if (state == EAST) {
    eastWall = frontWall;
    northWall = leftWall;
    southWall = rightWall;
    westWall = false;
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
  Serial.print("EncoderL Count: ");
  Serial.println(enCountsL);
  Serial.print("EncoderR Counts: ");
  Serial.println(enCountsR);
  Serial.println(" ");
}

// finds index of smallest value in an array of size 4
int findSmallest(unsigned char possibleMoves[]) {
   int smallest = 0;
   for (int k = 0; k < 4; k++) {
     if (possibleMoves[k] < possibleMoves[smallest]) {
        smallest = k;
     }
   }
   return smallest;
}

// from the internet - edited slightly for arduino
void floodMaze(unsigned char goal)
{

  unsigned char p, q;
  unsigned char now,next;
  // unsigned char passes;
  unsigned char cellwalls; // the wall data for a given cell  
  unsigned char changed;
  
  p = 0;
  do{
    mapx[p--] = 255;
  } while (p);
  
  mapx[goal] = 0;
  //passes = 0;
  now = 0;
  next = now + 1; // next = 1
  do
  {
    changed = 0;
    p = 0;
    do
    {
      if (mapx[p] == now)
      {
        cellwalls = maze[p];
        if ((cellwalls & NORTH) == 0) // if there isn't a wall to the north
        {
          q = p + 1;
          if (mapx[q] == 255) {
            mapx[q] = next;
            changed = 1;
          }
        }
        if ((cellwalls & EAST) == 0) // if no wall to the east
        {
          q = p + 16;
          if (mapx[q] == 255) {
            mapx[q] = next;
            changed = 1;
          }
        }
        if ((cellwalls & SOUTH) == 0) // if no wall to the south
        {
          q = p + 255;
          if (mapx[q] == 255) {
            mapx[q] = next;
            changed = 1;
          }
        }
        if ((cellwalls & WEST) == 0) // if no wall to the west
        {
          q = p + 240;
          if (mapx[q] == 255) {
            mapx[q] = next;
            changed = 1;
          }
        }
      }
      p--;
    } while(p);
    now  = now + 1;
    next = now + 1;
    //passes++;
  } while(changed);
  //return passes;
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



