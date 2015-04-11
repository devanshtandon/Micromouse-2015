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
int FmultiMap(int val, int * _in, int * _out, byte size)
{
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];
  if (val >= _in[size-1]) return _out[size-1];

  // search right interval
  byte pos = 1;  // _in[0] allready tested
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

/**
  *     PATHFINDING
  **/
  
#include "stdint.h"

#define NORTH      1
#define EAST       2
#define SOUTH      4
#define WEST       8           
#define VISITED      16
#define ONROUTE      32

#define ANORTH 0
#define AEAST 1
#define ASOUTH 2
#define AWEST 3

#define NWEST 4
#define NSOUTH 8
#define NEAST 1
#define NNORTH 2

struct coord {
  int x;
  int y;
};

class State
{
    public:
    struct coord *pos;
    int orientation; // N, E, S, W in that order
    
    State()
    {
        pos = (struct coord *) malloc(sizeof(*pos));
        pos->x = 0;
        pos->y = 0;
        orientation = 0;
    }
    
    ~State() {
        free(pos);
    }
    
    void
    update(int direction)
    {
        int increment;
        switch (direction) {
            case FORWARD:
            case BACKWARD:
                if(direction == FORWARD) increment = 1;
                else increment = -1;
                switch (orientation) {
                    case ANORTH:
                        pos->y += increment;
                        break;
                    case AEAST:
                        pos->x += increment;
                        break;
                    case ASOUTH:
                        pos->y -= increment;
                        break;
                    case AWEST:
                        pos->x -= increment;
                        break;
                }
                break;
            case LEFT:
                orientation = orientation - 1 % 4;
                break;
            case RIGHT:
                orientation = orientation + 1 % 4;
                break;
        }
    }
};

State *state = new State();
State *tempState = new State();

// maze storage
#define NUMCELLS 256

byte maze[NUMCELLS];
byte mazemap[NUMCELLS];
byte moveset[100];
int moves, movecounter;

byte floodMaze(byte goal)
{
  byte i,j;
  byte now,next;
  byte passes;
  byte cellwalls;    // the wall data for a given cell
  byte changed;
  Serial.println("Running floodMaze");
  i = 0;
  do{
    mazemap[i--] = 255;
  } while (i);

  mazemap[goal]=0;
  passes = 0;
  now = 0;
  next = now+1;
  do
  {
    changed = 0;
    i = 0;
    do
    {
      if (mazemap[i]==now)
      {
        cellwalls=maze[i];
        if ((cellwalls & NORTH) == 0)
        {
          j = i+1;
          if (mazemap[j] == 255){ mazemap[j] = next; changed=1;}
        }
        if ((cellwalls & EAST) == 0)
        {
          j = i + 16;
          if (mazemap[j] == 255){ mazemap[j] = next; changed=1;}
        }
        if ((cellwalls & SOUTH) == 0)
        {
          j = i + 255;
          if (mazemap[j] == 255){ mazemap[j] = next; changed=1;}
        }
        if ((cellwalls & WEST) == 0)
        {
          j = i + 240;
          if (mazemap[j] == 255){ mazemap[j] = next; changed=1;}
        }
      }
      i--;
    } while(i);
    now  = now+1;
    next = now+1;
    passes++;
  } while(changed);
  
  for(int y=15; y >= 0; y--) {
    for(int x=0; x < 16; x++) {
      if(mazemap[x+16*y] > 9) {
        Serial.print(mazemap[x+16*y]);
        Serial.print(" ");
      } else {
        Serial.print(mazemap[x+16*y]);
        Serial.print("  ");
      }
    }
    Serial.println("");
  }
  
  return passes;
}

int
nextMove(byte start, byte *moveset)
{
    byte current;
    byte i = 0;
    int moves = 0;
    current = mazemap[start];
    tempState->orientation = state->orientation;
    Serial.println("Running nextMove");
    while(current != 0) {
        if(start % 16 != 15 && mazemap[start+1] == current - 1) {
            Serial.println("Condition 1");
            start += 1;
            current -= 1;
            switch (tempState->orientation) {
              case ANORTH:
                moves = 2;
                moveset[i++] = RIGHT;
                moveset[i++] = FORWARD;
                tempState->update(RIGHT);
                tempState->update(FORWARD);
                break;
              case AEAST:
                moves = 1;
                moveset[i++] = FORWARD;
                tempState->update(FORWARD);
                break;
              case ASOUTH:
                moves = 2;
                moveset[i++] = LEFT;
                moveset[i++] = FORWARD;
                tempState->update(LEFT);
                tempState->update(FORWARD);
                break;
              case AWEST:
                moves = 3;
                moveset[i++] = LEFT;
                moveset[i++] = LEFT;
                moveset[i++] = FORWARD;
                tempState->update(LEFT);
                tempState->update(LEFT);
                tempState->update(FORWARD);
                break;
            }
        } else if(start % 16 != 0 && mazemap[start-1] == current - 1) {
            Serial.println("Condition 2");
            start -= 1;
            current -= 1;
            switch (tempState->orientation) {
              case ANORTH:
                moves = 2;
                moveset[i++] = LEFT;
                moveset[i++] = FORWARD;
                tempState->update(LEFT);
                tempState->update(FORWARD);
                break;
              case AEAST:
                moves = 3;
                moveset[i++] = LEFT;
                moveset[i++] = LEFT;
                moveset[i++] = FORWARD;
                tempState->update(LEFT);
                tempState->update(LEFT);
                tempState->update(FORWARD);
                break;
              case ASOUTH:
                moves = 2;
                moveset[i++] = RIGHT;
                moveset[i++] = FORWARD;
                tempState->update(RIGHT);
                tempState->update(FORWARD);
                break;
              case AWEST:
                moves = 1;
                moveset[i++] = FORWARD;
                tempState->update(FORWARD);
                break;
            }
        } else if(start < 240 && mazemap[start+16] == current - 1) {
            Serial.println("Condition 3");
            start += 16;
            current -= 1;
            switch (tempState->orientation) {
              case ANORTH:
                moves = 1;
                moveset[i++] = FORWARD;
                tempState->update(FORWARD);
                break;
              case AEAST:
                moves = 2;
                moveset[i++] = LEFT;
                moveset[i++] = FORWARD;
                tempState->update(LEFT);
                tempState->update(FORWARD);
                break;
              case ASOUTH:
                moves = 3;
                moveset[i++] = LEFT;
                moveset[i++] = LEFT;
                moveset[i++] = FORWARD;
                tempState->update(LEFT);
                tempState->update(LEFT);
                tempState->update(FORWARD);
                break;
              case AWEST:
                moves = 2;
                moveset[i++] = RIGHT;
                moveset[i++] = FORWARD;
                tempState->update(RIGHT);
                tempState->update(FORWARD);
                break;
            }
        } else if(start >= 16 && mazemap[start-16] == current - 1) {
            Serial.println("Condition 4");
            start -= 16;
            current -= 1;
            switch (tempState->orientation) {
              case ANORTH:
                moves = 3;
                moveset[i++] = LEFT;
                moveset[i++] = LEFT;
                moveset[i++] = FORWARD;
                tempState->update(LEFT);
                tempState->update(LEFT);
                tempState->update(FORWARD);
                break;
              case AEAST:
                moves = 2;
                moveset[i++] = RIGHT;
                moveset[i++] = FORWARD;
                tempState->update(RIGHT);
                tempState->update(FORWARD);
                break;
              case ASOUTH:
                moves = 1;
                moveset[i++] = FORWARD;
                tempState->update(FORWARD);
                break;
              case AWEST:
                moves = 2;
                moveset[i++] = LEFT;
                moveset[i++] = FORWARD;
                tempState->update(LEFT);
                tempState->update(FORWARD);
                break;
            }
        }
        Serial.print("Orientation: ");
        Serial.println(tempState->orientation);
    }
    
    moveset[i] = 0;
    
    Serial.println("Moveset: ");
    for(int j=0; moveset[j] != 0; j++) {
      Serial.println(moveset[j]);
    }
    return moves;
}


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


#define WALL_THRESHOLD (70)
#define DELAY_DEBUG 1000

boolean frontWall;
boolean rightWall;
boolean leftWall;

boolean northWall;
boolean eastWall;
boolean westWall;
boolean southWall;


bool adjustToFrontWall = false;

#define SEEN (4)
#define UNSEEN (5)

//should be GLOBAL maze
int randomNumber = random(2);
//should be global location
struct coord location;



int dir;
int goForward;


void setup() {
  location.x = 1;
location.y = 1;
dir = FORWARD;
goForward = 0;

  moves = 0;
  movecounter = 0;
  
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

  randomSeed(20);
  


  Serial.println ("SETUP COMPELETE");
  delay(2000);
}


void loop() {
  Serial.println("Initiating loop");
  location.x = 5;
  location.y = 5;
  floodFillMove(*state->pos, location);
  Serial.println("Finish loop");
  delay(1000);
}


//Give a direction (FORWARD, BACKWARD, LEFT, or RIGHT)
//and the number of encoder counts to move.
void go(int direction, int counts) {
  Serial.println("Going");
  centre();

  resetEncoders();
  setMotorSpeeds(spL,spR);
  setMotorDirection(direction);
  getSensors();
  boolean wallClose=false;

  // if (sensorValues[FRONT] < WALL_THRESHOLD) FRONT_WALL = true;
  // if (sensorValues[LEFT_FRONT] < WALL_THRESHOLD) LEFT_WALL = true;
  // if (sensorValues[RIGHT_FRONT] < WALL_THRESHOLD) RIGHT_WALL = true;

  if (direction == FORWARD) {

    // difference between forward left and right sensors
    int frontDiff=0;
    // difference between back left and right sensors
    int backDiff=0;
    int frontDiffAvg = 0;
    int backDiffAvg = 0;

    int counter = 0;
    int distToLeft = 0;
    int distToRight = 0;
    int distToLeftAvg = 0;
    int distToRightAvg = 0;
    // input for diffBack
    int input2 = 0;
    output = 0;
    
    while ( (enCountsL+enCountsR)/2 <counts && !wallClose) {  //1487
      getEncoders();
      detectWalls();
      unsigned long currentMillis = millis();
      if(currentMillis - previousMillis > 20) {
        // setup for PID
        previousMillis = currentMillis; // save the last time you blinked the LED
        delay(1);
        distToLeft += READ_SENSOR(LEFT_FRONT);
        distToRight += READ_SENSOR(RIGHT_FRONT);
        frontDiff += distToLeft - distToRight;
        backDiff += READ_SENSOR(LEFT_BACK)-READ_SENSOR(RIGHT_BACK);
        wallClose=checkWall();
        counter=counter+1;
        if (counter==3) {
          frontDiffAvg=frontDiff/3;
          backDiffAvg=backDiff/3;
          distToLeftAvg=distToLeft/3;
          distToRightAvg=distToRight/3;
          frontDiff=0;
          backDiff=0;
          distToLeft=0;
          distToRight=0;
          counter=0;
        }
      }

      if (leftWall==true && rightWall==true) {
        // 2 wall PID
        input = frontDiffAvg;
        myPID.SetMode(pidSwitch);
        digitalWrite(13, HIGH);
        digitalWrite(DEBUG,LOW);
      }
      else if (leftWall==true && rightWall==false) {
        // 1 wall PID 
        // close to left wall
        input = (distToLeftAvg - 32);
        myPID.SetMode(pidSwitch);
        digitalWrite(13, LOW);
        digitalWrite(DEBUG,HIGH);
      }
      else if (leftWall==false && rightWall==true) {
        // 1 wall PID
        // close to right wall
        input = - (distToRightAvg - 32);
        myPID.SetMode(pidSwitch);
        digitalWrite(13, LOW);
        digitalWrite(DEBUG,HIGH);
      }
      else {
        // we're fucked
        // not 2 wall PID
        // PID goes off
        myPID.SetMode(MANUAL);
        digitalWrite(13, LOW);
        digitalWrite(DEBUG,LOW);
      }

      myPID.Compute(); 
      setMotorSpeeds(constrain(spL+output,0,255),constrain(spR-output,0,255));
    }   

    stopRobot();
    digitalWrite(DEBUG,LOW);
    digitalWrite(13, LOW);
  }

  else {
    // go forward till it is close enough to front wall to turn
    while(!wallClose && adjustToFrontWall) {
      // int diff=0;
      // diff += READ_SENSOR(LEFT_FRONT)-READ_SENSOR(RIGHT_FRONT);
      // myPID.Compute(); 
      setMotorDirection(FORWARD);
      setMotorSpeeds(constrain(spL,0,255),constrain(spR,0,255));
      wallClose = checkWall();
    }

    adjustToFrontWall = false;
    setMotorDirection(direction);
    while(abs(enCountsL)<counts) {
      enCountsL = encoders.getCountsM1();
      enCountsR = encoders.getCountsM2();
    }

  }

  stopRobot();
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
  go(FORWARD, 1*SQUARE);
}

void backwardOneSquare() {
  go(BACKWARD, 1*SQUARE);
}

void turnRight() {
  go(RIGHT, 1*TURN);
}

void turnLeft() {
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


// update the sensorValues array
void getSensors() {

  sensorValues[LEFT_BACK] = READ_SENSOR(LEFT_BACK);
  sensorValues[LEFT_FRONT] = READ_SENSOR(LEFT_FRONT);
  sensorValues[FRONT] = READ_SENSOR(FRONT);
  sensorValues[RIGHT_FRONT] = READ_SENSOR(RIGHT_FRONT);
  sensorValues[RIGHT_BACK] = READ_SENSOR(RIGHT_BACK);

}

// checks if the front wall is too close
// stop from running into front wall
boolean checkWall() {
  if(READ_SENSOR(FRONT) < 37)
    return true;
  else
    return false;
}

void centre() {
  detectWalls();
  if (leftWall == true) {
    while (READ_SENSOR(LEFT_FRONT) - READ_SENSOR(LEFT_BACK) > 2) {
      setMotorDirection(LEFT);
      setMotorSpeeds(10,10);
    }
    while (READ_SENSOR(LEFT_BACK) - READ_SENSOR(LEFT_FRONT) > 2) {
      setMotorDirection(RIGHT);
      setMotorSpeeds(10,10);
    }
  }
  else if (rightWall == true) {
    while (READ_SENSOR(RIGHT_FRONT) - READ_SENSOR(RIGHT_BACK) > 2) {
      setMotorDirection(RIGHT);
      setMotorSpeeds(10,10);
    }
    while (READ_SENSOR(RIGHT_BACK) - READ_SENSOR(RIGHT_FRONT) > 2) {
      setMotorDirection(LEFT);
      setMotorSpeeds(10,10);
    }
  }
  else {
    // no centre-ing possible
    // you are fucked.
  }
  stopRobot();
  delay(500);
}


// checks the walls, sets the boolean array
void detectWalls() {
    getSensors();
    
    if (sensorValues[FRONT] < WALL_THRESHOLD) {
        frontWall= true;
    } else {
        frontWall=false;
    }
    
    if (sensorValues[LEFT_FRONT] < WALL_THRESHOLD && sensorValues[LEFT_BACK] < WALL_THRESHOLD) {
        leftWall= true;
    } else {
        leftWall=false;
    }
    
    if (sensorValues[RIGHT_FRONT] < WALL_THRESHOLD && sensorValues[RIGHT_BACK] < WALL_THRESHOLD) {
        rightWall=true;
    } else {
        rightWall=false;
    }
    
    switch (state->orientation) {
        case ANORTH:
            northWall = frontWall;
            eastWall = rightWall;
            westWall = leftWall;
            southWall = false;
            break;
        case ASOUTH:
            southWall = frontWall;
            eastWall = leftWall;
            westWall = rightWall;
            northWall = false;
            break;
        case AWEST:
            westWall = frontWall;
            northWall = rightWall;
            southWall = leftWall;
            eastWall = false;
            break;
        case AEAST:
            eastWall = frontWall;
            northWall = leftWall;
            southWall = rightWall;
            westWall = false;
            break;
    }
}

void
floodFillMove(struct coord start, struct coord finish)
{
    byte wall, cstart, cfinish;
    
    Serial.print("position.x: ");
    Serial.println(start.x);
    Serial.print("position.y: ");
    Serial.println(start.y);
    Serial.println("orientation.y: ");
    Serial.println(state->orientation);
    
    detectWalls();
    
    Serial.println("Running floodfillMove");
    wall = 0;
    if(northWall) {
      wall += NNORTH;
      Serial.println("Found north wall");
    }
    if(southWall) {
      wall += NSOUTH;
      Serial.println("Found south wall");
    }
    if(eastWall) {
      wall += NEAST;
      Serial.println("Found east wall");
    }
    if(westWall) {
      wall += NWEST;
      Serial.println("Found west wall");
    }
    maze[start.x + 16*(start.y)] = wall;
    
    cfinish = (finish.x + 16*(finish.y));
    
    cstart = (start.x + 16*(start.y)); 
    
    if(moves <= 0) {
      floodMaze(cfinish);
      moves = nextMove(cstart, moveset);
      movecounter = 0;
    } else {
      moves--;
    }
    
    if(frontWall) adjustToFrontWall = true;
    
    if(moveset[movecounter] == 0) {
        Serial.println("DONE!");
    }else if(moveset[movecounter] == FORWARD) {
        Serial.println("Move forward");
        go(FORWARD, 1*SQUARE);
        state->update(FORWARD);
    } else if(moveset[movecounter] == BACKWARD) {
        Serial.println("Move backward");
        turnAround();
    } else if(moveset[movecounter] == LEFT) {
        Serial.println("Move left");
        go(LEFT, 1*TURN);
        state->update(LEFT);
    } else if(moveset[movecounter] == RIGHT) {
        Serial.println("Move right");
        go(RIGHT, 1*TURN);
        state->update(RIGHT);
    }
    movecounter++;
        
    return;
}
