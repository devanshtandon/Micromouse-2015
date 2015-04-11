/* micromouse2015.ino 
Christopher Datsikas
Devansh Tandon
Created: 03-16-2015
Last Modified: 03-22-2015
Status: In Progress
Arduino Code for Micromouse for 
2015 Brown IEEE Robotics Competition
Github: https://urldefense.proofpoint.com/v2/url?u=https-3A__github.com_devanshtandon_Micromouse-2D2015&d=AwIGAg&c=-dg2m7zWuuDZ0MUcV7Sdqw&r=CMK86WddPzKBR8aS9xUfCIpO4VcEDRH-I4XgL4HK2Lc&m=2oUcaJb4DGLCRDrkvWxJD3UdUtEyZY7NdXYuwwy2FCs&s=9JgUlRVcDU1LnZI1K7K10kEQWIZQsUSl_HR7oUfGNwg&e= 
Upverter: https://urldefense.proofpoint.com/v2/url?u=https-3A__upverter.com_chrisdats_26efdb039ba46d67_Micromouse2015_&d=AwIGAg&c=-dg2m7zWuuDZ0MUcV7Sdqw&r=CMK86WddPzKBR8aS9xUfCIpO4VcEDRH-I4XgL4HK2Lc&m=2oUcaJb4DGLCRDrkvWxJD3UdUtEyZY7NdXYuwwy2FCs&s=Lwi2bWaXUhpoOyBSn-f1X2hNvc-TnL4aclxY-A7cPK8&e= 
Components:
- TB6612FNG Dual Motor Driver Carrier - https://urldefense.proofpoint.com/v2/url?u=https-3A__www.pololu.com_product_713&d=AwIGAg&c=-dg2m7zWuuDZ0MUcV7Sdqw&r=CMK86WddPzKBR8aS9xUfCIpO4VcEDRH-I4XgL4HK2Lc&m=2oUcaJb4DGLCRDrkvWxJD3UdUtEyZY7NdXYuwwy2FCs&s=jeuuFd2xX9PDgS-1VF7hRSbklv429eljQyxoyXuHWWI&e= 
- 75:1 Micro Metal Gearmotor HP with Extended Motor Shaft - https://urldefense.proofpoint.com/v2/url?u=https-3A__www.pololu.com_product_2215&d=AwIGAg&c=-dg2m7zWuuDZ0MUcV7Sdqw&r=CMK86WddPzKBR8aS9xUfCIpO4VcEDRH-I4XgL4HK2Lc&m=2oUcaJb4DGLCRDrkvWxJD3UdUtEyZY7NdXYuwwy2FCs&s=6VGdtzYqEOtr8TtMUZCAEDkKMOuaF7GhKiYh4D-MRRM&e= 
- Sharp GP2Y0A51SK0F Analog Distance Sensor 2-15cm - https://urldefense.proofpoint.com/v2/url?u=https-3A__www.pololu.com_product_2450&d=AwIGAg&c=-dg2m7zWuuDZ0MUcV7Sdqw&r=CMK86WddPzKBR8aS9xUfCIpO4VcEDRH-I4XgL4HK2Lc&m=2oUcaJb4DGLCRDrkvWxJD3UdUtEyZY7NdXYuwwy2FCs&s=30C5odkdmPkMPxYhFAmJK2suDeztJdkqX5hug7kG3dg&e= 
- Magnetic Encoders 12 CPR https://urldefense.proofpoint.com/v2/url?u=https-3A__www.pololu.com_product_2598&d=AwIGAg&c=-dg2m7zWuuDZ0MUcV7Sdqw&r=CMK86WddPzKBR8aS9xUfCIpO4VcEDRH-I4XgL4HK2Lc&m=2oUcaJb4DGLCRDrkvWxJD3UdUtEyZY7NdXYuwwy2FCs&s=gE7qdD7jFtKh2iX5LXW919sqjfybxeSkkJ8YglZGRgA&e= 
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


// out[] holds the values wanted in mm
int out[] = {
  300, 210, 190, 150, 140, 130, 120, 110, 100, 90, 80, 70, 60, 50, 40, 30, 25, 20, 15, 10};
// in[] holds the measured analogRead() values for defined distances
int in[]  = {
  72, 96, 100, 113, 116, 125, 129, 132, 137, 141, 148, 159, 181, 212, 256, 320, 360, 423, 500, 570};

//-----------------------------------------------------------


// Define pins for Motor Driver TB6612FNG
#define PWMA 5
#define PWMB 6

#define AIN2 2
#define AIN1 3
#define BIN1 7
#define BIN2 8

#define STBY 4

#define DEBUG (A5)

// Define directions
#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4


// IR Sensor Setup
#define LEFT_BACK (4)
#define LEFT_FRONT (3)
#define FRONT (2)
#define RIGHT_FRONT (1)
#define RIGHT_BACK (0)
#define CALIBRATION_ARRAY_SIZE 20
#define READ_SENSOR(c) (FmultiMap(analogRead(c), in, out, CALIBRATION_ARRAY_SIZE))
double sensorValues[5];
//boolean walls[4] = {false, false, false, false};
//#define LEFT_WALL walls[0]
//#define FRONT_WALL walls[1]
//#define RIGHT_WALL walls[2]
//#define BACK_WALL walls[3]
//// 0 -- Left
//// 1 -- Front
//// 2 -- Right
//// 2 -- Back


// MOTOR CONTROL CONSTANTS: NEEDS FINE-TUNING
const int COUNTS_PER_CM=93; 
const int SQUARE=1780;  // 1750
const int TURN=780;     // 795
double spL = 74;
double spR = 73;

// PID Setup
#include <PID_v1.h>
double setpoint = 0, input, output = 0;
long previousMillis = 0;
PID myPID(&input, &output, &setpoint,.20,.10,.05, DIRECT);
boolean pidSwitch=AUTOMATIC;


// Encoder Setup
#include <PololuWheelEncoders.h>
PololuWheelEncoders encoders;
double enCountsL; // encoder counts
double enCountsR;
#define WALL_THRESHOLD (60)




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
  
  Serial.begin(9600);
  PololuWheelEncoders::init(10,9,12,11);

  // set all the Motor Driver pins to OUTPUT
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);  // turns motor driver on
  
  pinMode(DEBUG, OUTPUT);
  digitalWrite(DEBUG, LOW);

  // set up the PID
  myPID.SetSampleTime(100);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetMode(pidSwitch);
  
     //fill maze and mapx with 0's
  for (int k = 0; k < NUMCELLS; k++) {
    maze[k] = 0;
    mapx[k] = 0;
  }

  maze[0] += SOUTH;

  Serial.println ("SETUP COMPELETE");
  delay(2000);
}


void loop() {
  
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

}



//Give a direction (FORWARD, BACKWARD, LEFT, or RIGHT)
//and the number of encoder counts to move.
void go(int direction, int counts) {

  resetEncoders();
  setMotorSpeeds(spL,spR);
  setMotorDirection(direction);
  getSensors();
  boolean wallClose=false;

  // if (sensorValues[FRONT] < WALL_THRESHOLD) FRONT_WALL = true;
  // if (sensorValues[LEFT_FRONT] < WALL_THRESHOLD) LEFT_WALL = true;
  // if (sensorValues[RIGHT_FRONT] < WALL_THRESHOLD) RIGHT_WALL = true;

  if (direction == FORWARD) {

    int diff=0;
    int diffBack=0;
    int counter = 0;
    int input2 = 0;
    output = 0;
    while ( (enCountsL+enCountsR)/2 <counts && !wallClose) {  //1487
      getEncoders();
      unsigned long currentMillis = millis();
      if(currentMillis - previousMillis > 20) {
        previousMillis = currentMillis; // save the last time you blinked the LED
        delay(1);
        diff += READ_SENSOR(LEFT_FRONT)-READ_SENSOR(RIGHT_FRONT);
        diffBack += READ_SENSOR(LEFT_BACK)-READ_SENSOR(RIGHT_BACK);
        wallClose=checkWall();
        counter=counter+1;
        if (counter==3) {
          input=diff/3;
          input2=diffBack/3;
          diff=0;
          diffBack=0;
          counter=0;
        }
      }
      if (abs(input)>40 || abs(input2)>40) {
        myPID.SetMode(MANUAL);
        digitalWrite(DEBUG,LOW);
      }
      else {
        myPID.SetMode(pidSwitch);
        digitalWrite(DEBUG,HIGH);
      }
      myPID.Compute(); 
      setMotorSpeeds(constrain(spL+output,0,255),constrain(spR-output,0,255));
    }   

    stopRobot();
    digitalWrite(DEBUG,LOW);

  }

  else {
    // go forward till it is close enough to front wall to turn
    while(!wallClose) {
      // int diff=0;
      // diff += READ_SENSOR(LEFT_FRONT)-READ_SENSOR(RIGHT_FRONT);
      // myPID.Compute(); 
      setMotorDirection(FORWARD);
      setMotorSpeeds(constrain(spL,0,255),constrain(spR,0,255));
      wallClose = checkWall();
    }

    setMotorDirection(direction);
    while(abs(enCountsL)<counts) {
      enCountsL = encoders.getCountsM1();
      enCountsR = encoders.getCountsM2();
    }

  }

  stopRobot();
}


void wallFollow() {

  while(1) {
    if (!checkWall())
      forwardOneSquare();
    else {
      getSensors();
      if (sensorValues[LEFT_FRONT] > 45 && sensorValues[LEFT_BACK] > 45)
        turnLeft();
      else if (sensorValues[RIGHT_FRONT] > 45 && sensorValues[RIGHT_BACK] > 45)
        turnRight();
      else
        turnAround();
    }
    delay(1000);
  }

}













void setMotorDirection(int direction) {
  if (direction == FORWARD) {
    digitalWrite(AIN1, LOW);   // Left wheel forward
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);  // Right wheel forward
    digitalWrite(BIN2, LOW);  
  }

  else if (direction == BACKWARD) {
    digitalWrite(AIN1, HIGH); // Left wheel reverse
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);  // Right wheel reverse
    digitalWrite(BIN2, HIGH);
  }

  else if (direction == LEFT) {
    digitalWrite(AIN1, HIGH);  // Left wheel reverse
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);  // Right wheel forward
    digitalWrite(BIN2, LOW);
  }

  else if (direction == RIGHT) {
    digitalWrite(AIN1, LOW);  // Left wheel forward
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW);  // Right wheel reverse
    digitalWrite(BIN2, HIGH);
  }
}


void setMotorSpeeds(double spL, double spR) {
  analogWrite(PWMA,spL);
  analogWrite(PWMB,spR);
}


// stop both motors and bring robot to a stop
void stopRobot() {
  digitalWrite(AIN1, HIGH);  // stop left wheel
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);  // stop right wheel
  digitalWrite(BIN2, HIGH);  
}

void forwardOneSquare() {
  if (state == NORTH) {
    i = i + 1;
  } else if (state == SOUTH) {
    i = i - 1;
  } else if (state == WEST) {
    i = i - 16;
  } else if (state == EAST) {
    i = i + 16;
  }
  go(FORWARD, 1*SQUARE);
}

void backwardOneSquare() {
  if (state == NORTH) {
    i = i - 1;
  } else if (state == SOUTH) {
    i = i + 1;
  } else if (state == WEST) {
    i = i + 16;
  } else if (state == EAST) {
    i = i - 16;
  }
  go(BACKWARD, 1*SQUARE);
}

void turnRight() {
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
  go(RIGHT, 1*TURN);
}

void turnLeft() {
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
  go(LEFT, 1*TURN);
}

void turnAround() {
  go(LEFT, 1*TURN);
  delay(1000);
  go(LEFT, 1*TURN);
}

void getEncoders(){
  enCountsL=encoders.getCountsM1();
  enCountsR=encoders.getCountsM2();
}

void resetEncoders() {
  encoders.getCountsAndResetM1();
  encoders.getCountsAndResetM2();
  enCountsL = 0;
  enCountsR = 0;
}


void getSensors() {

  sensorValues[LEFT_BACK] = READ_SENSOR(LEFT_BACK);
  sensorValues[LEFT_FRONT] = READ_SENSOR(LEFT_FRONT);
  sensorValues[FRONT] = READ_SENSOR(FRONT);
  sensorValues[RIGHT_FRONT] = READ_SENSOR(RIGHT_FRONT);
  sensorValues[RIGHT_BACK] = READ_SENSOR(RIGHT_BACK);

}


boolean checkWall() {
  if(READ_SENSOR(FRONT) < 37)
    return true;
  else
    return false;
}
