/* micromouse2015.ino 

Devansh Tandon
Henry Li
Richard Chang
Alex Ringlein
Christopher Datsikas
Bernardo Savaria

Created: 03-16-2015
Last Modified: 03-22-2015
Status: Complete
Arduino Code for Micromouse for 
2015 Brown IEEE Robotics Competition

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

// Define micromouse instructions
#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define ENDSTEP 5

/*
Maze Constants:
*/

// Define cardinal directions for floodfill
#define FNORTH      2
#define FEAST       1
#define FSOUTH      8
#define FWEST       4

// Define cardinal directions for moveset func
#define DNORTH      0
#define DEAST       1
#define DSOUTH      2
#define DWEST       3

// maze storage
#define NUMCELLS 256

// maze variables
unsigned char maze[NUMCELLS];
unsigned char mazemap[NUMCELLS];
unsigned char moveset[NUMCELLS];

// IR Sensor Setup
#define LEFT_BACK (4)
#define LEFT_FRONT (3)
#define FRONT (2)
#define RIGHT_FRONT (1)
#define RIGHT_BACK (0)
#define CALIBRATION_ARRAY_SIZE 20
#define READ_SENSOR(c) (FmultiMap(analogRead(c), in, out, CALIBRATION_ARRAY_SIZE))
double sensorValues[5];


// MOTOR CONTROL CONSTANTS:
const int COUNTS_PER_CM=93; 
const int SQUARE=1750;  
const int TURN=780;     
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

struct coord {
  unsigned char x;
  unsigned char y;
};

#define SEEN (4)
#define UNSEEN (5)

unsigned char orientation = DNORTH;
struct coord location;
struct coord finish;

int goForward;


void setup() {
    location.x = 0;
    location.y = 0;
    finish.x = 8;
    finish.y = 8;
    orientation = DNORTH;

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
    
    unsigned char wall;
    
    for(int j=0; j < 16; j++) {
        for(int i=0; i < 16; i++) {
            wall = 0;
            if(i == 0) wall += FEAST + FWEST;
            if(j == 0) wall += FSOUTH + FNORTH;
            if(i == 15) wall += FWEST + FEAST;
            if(j == 15) wall += FNORTH + FSOUTH;
            if(i == 10) wall = 0;

            maze[i + 16*j] = wall;
        }
    }

    Serial.println ("SETUP COMPELETE");
    delay(200);
}


void loop() {
    detectWalls();
    runFloodfill();

    for(int i=0; moveset[i] != ENDSTEP; i++) {
        switch(moveset[i]) {
            case LEFT:
                turnLeft();
                break;
            case RIGHT:
                turnRight();
                break;
            case FORWARD:
                forwardOneSquare();
                break;
        }
    }
}


//updating when moving forward
void updateForward() {
  if (orientation == DNORTH) {
    location.y++;
  } else if (orientation == DSOUTH) {
    location.y--;
  } else if (orientation == DWEST) {
    location.x--;
  } else if (orientation == DEAST) {
    location.x++;
  }
}

//updating when turning right
void updateRight() {
    orientation = (unsigned char) (((int) orientation + 1) % 4);
}
//update when turning  left
void updateLeft() {
    orientation = (unsigned char) (((int) orientation - 1) % 4);
}
//updating when turning around
void updateTurnAround() {
    orientation = (unsigned char) (((int) orientation + 2) % 4);
}


//Give a direction (FORWARD, BACKWARD, LEFT, or RIGHT)
//and the number of encoder counts to move.
void go(int direction, int counts) {

  centre();

  resetEncoders();
  setMotorSpeeds(spL,spR);
  setMotorDirection(direction);
  getSensors();
  boolean wallClose=false;

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
    
    while ( (enCountsL+enCountsR)/2 <counts && !wallClose) {  
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


// simple algorithm
void runFloodfill() {
    for(int i=0; i < NUMCELLS; i++) {
        moveset[i] = 0;
    }

    maze[location.x + 16*location.y] = 0;

    if (frontWall) {
        adjustToFrontWall = true;
        maze[location.x + 16*location.y] += pow(2, orientation);
    }
    if (leftWall) {
        maze[location.x + 16*location.y] += (unsigned char) pow(2, ((int) orientation - 1) % 4);
    }
    if (rightWall) {
        maze[location.x + 16*location.y] += (unsigned char) pow(2, ((int) orientation + 1) % 4);
    }

    unsigned char passes = floodMaze(finish.x + 16*finish.y);

    createMoveset((location.x + 16*location.y), moveset);

    delay(200);
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
  updateForward();
}

void backwardOneSquare() {
  go(BACKWARD, 1*SQUARE);
}

void turnRight() {
  go(RIGHT, 1*TURN);
  updateRight();
}

void turnLeft() {
  go(LEFT, 1*TURN);
  updateLeft();
}

void turnAround() {
  go(LEFT, 1*TURN);
  delay(200);
  go(LEFT, 1*TURN);
  updateTurnAround();
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
    // you are screwed.
    // good luck.
  }
  stopRobot();
  delay(200);
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
}

unsigned char floodMaze(unsigned char goal)
{
  unsigned char i,j;
  unsigned char now,next;
  unsigned char passes;
  unsigned char cellwalls;    // the wall data for a given cell
  unsigned char changed;

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
        if (i % 16 != 15 && !(maze[i] & FEAST) && !(maze[i+1] & FWEST))
        {
          j = i + 1;
          if (mazemap[j] == 255){ mazemap[j] = next; changed=1;}
        }
        if (i < 240 && !(maze[i] & FNORTH) && !(maze[i+16] & FSOUTH))
        {
          j = i + 16;
          if (mazemap[j] == 255){ mazemap[j] = next; changed=1;}
        }
        if (i % 16 != 0 && !(maze[i] & FWEST) && !(maze[i-1] & FEAST))
        {
          j = i - 1;
          if (mazemap[j] == 255){ mazemap[j] = next; changed=1;}
        }
        if (i >= 16 && !(maze[i] & FSOUTH) && !(maze[i-16] & FNORTH))
        {
          j = i - 16;
          if (mazemap[j] == 255){ mazemap[j] = next; changed=1;}
        }
      }
      i--;
    } while(i);
    now  = now+1;
    next = now+1;
    passes++;
  } while(changed);
  return passes;
}

void
nextMove(unsigned char direction, unsigned char *orientationP, unsigned char **movesetP, unsigned char *iP) {
    signed char move;
    unsigned char *moveset = *movesetP;
    move = (int) *orientationP - (int) direction;
    switch(move) {
        case 0:
            moveset[*iP] = FORWARD;
            *iP += 1;
            break;
        case -3:
        case 1:
            moveset[*iP] = LEFT;
            *iP += 1;
            *orientationP = (*orientationP - 1 % 4);
            moveset[*iP] = FORWARD;
            *iP += 1;
            break;
        case -2:
        case 2:
            moveset[*iP] = RIGHT;
            *iP += 1;
            *orientationP = (*orientationP + 1 % 4);
        case 3:
        case -1:
            moveset[*iP] = RIGHT;
            *iP += 1;
            *orientationP = (*orientationP + 1 % 4);
            moveset[*iP] = FORWARD;
            *iP += 1;
            break;
            
    }
    moveset[*iP] = ENDSTEP;
    *iP += 1;
}

void
createMoveset(unsigned char start, unsigned char *moveset)
{
    unsigned char current;
    unsigned char i;
    current = mazemap[start];
    
    i=0;
    while(current != 0) {
        if(start % 16 != 15 && !(maze[start] & FEAST) && mazemap[start+1] == current - 1) { // go east
            start += 1;
            current -= 1;
            nextMove(DEAST, &orientation, &moveset, &i);
        } else if(start % 16 != 0 && !(maze[start] & FWEST) && mazemap[start-1] == current - 1) { // go west
            start -= 1;
            current -= 1;
            nextMove(DWEST, &orientation, &moveset, &i);
        } else if(start < 240 && !(maze[start] & FNORTH) && mazemap[start+16] == current - 1) { // go north
            start += 16;
            current -= 1;
            nextMove(DNORTH, &orientation, &moveset, &i);
        } else if(start >= 16 && !(maze[start] & FSOUTH) && mazemap[start-16] == current - 1) { // go south
            start -= 16;
            current -= 1;
            nextMove(DSOUTH, &orientation, &moveset, &i);
        } else {
            moveset[0] = 0;
            printf("MOVESET:\nNo path to target!\n");
            return;
        }
    }
    
    moveset[i] = 0;
    
    printf("MOVESET:\n");
    
    for(int i=0; moveset[i] != 0; i++) {
        switch(moveset[i]) {
            case LEFT:
                printf("LEFT\n");
                break;
            case RIGHT:
                printf("RIGHT\n");
                break;
            case FORWARD:
                printf("FORWARD\n");
                break;
            case ENDSTEP:
                printf("END STEP\n");
                break;
        }
    }
    
    return;
}


