/* micromouse2015.ino 
Christopher Datsikas
Devansh Tandon
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

/**
 * PATHFINDING
 **/

//Structs
struct coord
{
    int x;
    int y;
};

struct node
{
    int x;
    int y;
    bool direction[4]; // N, E, S, W in that order
    
    int distance;
    struct node *previous;
};

class State
{
    struct coord *pos;
    int orientation; // N, E, S, W in that order
    
    State()
    {
        pos = malloc(sizeof(*pos));
        pos->x = 0;
        pos->y = 0;
        orientation = 0;
    }
    
    ~State() {
        free(pos);
    }
    
    void
    update(direction)
    {
        int increment;
        switch (direction) {
            case FORWARD:
            case BACKWARD:
                if(direction == FORWARD) increment = 1;
                else increment = -1;
                switch (orientation) {
                    case NORTH:
                        pos->y += increment;
                        break;
                    case EAST:
                        pos->x += increment;
                        break;
                    case SOUTH:
                        pos->y -= increment;
                        break;
                    case WEST:
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

// CONSTANTS

// finds child 0 or child 1 of x (dir = 0 or 1, respectively)
#define Child(x, dir) (2*(x)+1+(dir))

//finds parent of x
#define Parent(x) (((x)-1)/2)

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

#define START_SIZE (256)
#define INFINITY (255)
#define MAZE_WIDTH (16)
#define MAZE_HEIGHT (16)
#define PATH_LENGTH (100)

class PriorityQueue
{
private:
    int size;
    int n;
    struct node **p;
    
public:
    PriorityQueue(int newSize)
    {
        this->size = newSize;
        this->n = 0;
        this->p = (struct node **) malloc(sizeof(*this->p)*newSize);
    }
    
    ~PriorityQueue()
    {
        free(this->p);
    }
    
    void
    enqueue(struct node *node)
    {
        this->p[this->n] = node;
        floatUp(this->n++);
        
        if(this->n >= this->size) {
            this->size *= 2;
            this->p = (struct node **)realloc(this->p, sizeof(struct node *)*(this->size));
        }
    }
    
    struct node*
    dequeue(void)
    {
        struct node *x;
        
        x = this->p[0];
        this->p[0] = this->p[(this->n) - 1];
        this->n -= 1;
        floatDown(0);
        
        return x;
    }
    
    int
    returnSize(void) {
        return this->n;
    }
    
    void
    floatUp(int pos)
    {
        struct node *x, **a;
        a = this->p;
        
        x = a[pos];
        
        while(1) {
            if(pos > 0 && a[Parent(pos)]->distance > x->distance) {
                a[pos] = a[Parent(pos)];
                pos = Parent(pos);
            } else {
                break;
            }
        }
        
        a[pos] = x;
    }
    
    void
    floatDown(int pos)
    {
        struct node *x, **a;
        int n = this->n;
        a = this->p;
        
        x = a[pos];
        
        while(1) {
            if(Child(pos, 1) < n && a[Child(pos, 1)]->distance < a[Child(pos, 0)]->distance) {
                if(a[Child(pos, 1)]->distance < x->distance) {
                    a[pos] = a[Child(pos, 1)];
                    pos = Child(pos, 1);
                } else {
                    break;
                }
            } else if(Child(pos, 0) < n && a[Child(pos, 0)]->distance < x->distance) {
                a[pos] = a[Child(pos, 0)];
                pos = Child(pos, 0);
            } else {
                break;
            }
        }
        
        a[pos] = x;
    }
    
};

class Graph
{
private:
    struct node ***nodes;
    PriorityQueue *pqueue;
    
public:
    Graph()
    {
        pqueue = new PriorityQueue(START_SIZE);
        // initialize two dimensional graph
        this->nodes = (struct node ***) calloc(MAZE_WIDTH, sizeof(*(this->nodes)));
        for(int i=0; i<MAZE_WIDTH; i++) {
            this->nodes[i] = (struct node **) calloc(MAZE_HEIGHT, sizeof(*(this->nodes[i])));
            for(int j=0; j<MAZE_HEIGHT; j++) {
                this->nodes[i][j] = (struct node *) malloc(sizeof(*(this->nodes[i][j])));
            }
        }
    }
    
    ~Graph()
    {
        for(int i=0; i<MAZE_WIDTH; i++) {
            free(this->nodes[i]);
        }
        free(this->nodes);
        delete pqueue;
    }
    
    void
    addNode(int x, int y, bool north, bool east, bool south, bool west) {
        this->nodes[x][y]->x = x;
        this->nodes[x][y]->y = y;
        
        this->nodes[x][y]->direction[0] = north;
        this->nodes[x][y]->direction[1] = east;
        this->nodes[x][y]->direction[2] = south;
        this->nodes[x][y]->direction[3] = west;
    }
    
    coord **
    shortestPath(coord start, coord finish) {
        int i, alt;
        struct coord neighbor, swap, **path, *pathpt;
        struct node* smallest;
        
        for(int j = 0; j < MAZE_WIDTH; j++) {
            for(int i = 0; i < MAZE_HEIGHT; i++) {
                if(i == start.x && j == start.y) {
                    this->nodes[i][j]->distance = 0;
                    this->nodes[i][j]->previous = 0;
                    pqueue->enqueue(this->nodes[i][j]);
                } else {
                    this->nodes[i][j]->distance = INFINITY;
                    this->nodes[i][j]->previous = 0;
                    pqueue->enqueue(this->nodes[i][j]);
                }
            }
        }
        
        while(pqueue->returnSize() > 0) {
            smallest = pqueue->dequeue();
            
            if(smallest->x == finish.x && smallest->y == finish.y) {
                path = (struct coord **) malloc(sizeof(*path)*PATH_LENGTH);
                for(i = 0; smallest->previous != 0; i++) {
                    pathpt = (struct coord *) malloc(sizeof(*pathpt));
                    pathpt->x = smallest->x;
                    pathpt->y = smallest->y;
                    path[i] = pathpt;
                    smallest = smallest->previous;
                }
                
                // reverse path array
                for(int j=0; j<((i-1)/2); j++) {
                    swap = *path[j];
                    *path[j] = *path[i-j-1];
                    *path[i-j-1] = swap;
                }
                
                for(; i < PATH_LENGTH; i++) {
                    path[i] = NULL;
                }
                break;
            }
            
            if(!smallest || smallest->distance == INFINITY) {
                continue;
            }
            
            for(int i = 0; i < 4; i++) {
                if(!smallest->direction[i]) {
                    switch (i) {
                            // no north wall
                        case 0:
                            neighbor.x = smallest->x;
                            neighbor.y = smallest->y + 1;
                            break;
                            // no east wall
                        case 1:
                            neighbor.x = smallest->x + 1;
                            neighbor.y = smallest->y;
                            break;
                            // no south wall
                        case 2:
                            neighbor.x = smallest->x;
                            neighbor.y = smallest->y - 1;
                            break;
                            // no west wall
                        case 3:
                            neighbor.x = smallest->x - 1;
                            neighbor.y = smallest->y;
                            break;
                    }
                    
                    alt = smallest->distance + 1;
                    
                    if(alt < this->nodes[neighbor.x][neighbor.y]->distance) {
                        this->nodes[neighbor.x][neighbor.y]->distance = alt;
                        this->nodes[neighbor.x][neighbor.y]->previous = smallest;
                        pqueue->enqueue(this->nodes[neighbor.x][neighbor.y]);
                    }
                }
            }
        }
        return path;
    }
};


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
#define DELAY_DEBUG 1000


// IR Sensor Setup
#define LEFT_BACK (4)
#define LEFT_FRONT (3)
#define FRONT (2)
#define RIGHT_FRONT (1)
#define RIGHT_BACK (0)
#define CALIBRATION_ARRAY_SIZE 20
#define READ_SENSOR(c) (FmultiMap(analogRead(c), in, out, CALIBRATION_ARRAY_SIZE))
#define WALL_THRESHOLD 70

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

//Pathfinding Setup
State *state = new State();
Graph *maze = new Graph();
struct coord finish;

finish.x = 5;
finish.y = 5;

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

  Serial.println ("SETUP COMPELETE");
  delay(2000);
}


void loop() {
  
    detectWalls();
    nextMove(state->pos, finish);
//   wallFollow();
//   turnLeft();
//   delay(2000);

//   forwardOneSquare();
//   delay(2000);
//   forwardOneSquare();
//   delay(2000);
//   forwardOneSquare();
//   delay(2000);
//   forwardOneSquare();
//   delay(2000);
//   turnLeft();
//   delay(2000);
//   forwardOneSquare();
//   delay(2000);
//   turnLeft();
//   delay(2000);

//   forwardOneSquare();
// //  delay(2000);
//   forwardOneSquare();
// //  delay(2000);
//   forwardOneSquare();
// //  delay(2000);
//   forwardOneSquare();
// //  delay(2000);
//   turnLeft();
// //  delay(2000);
//   forwardOneSquare();
// //  delay(2000);
//   turnLeft();
// //  delay(2000);

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
    delay(DELAY_DEBUG);
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
  delay(DELAY_DEBUG);
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


void detectWalls() {
    
    if (sensorValues[FRONT] < WALL_THRESHOLD) {
        frontWall= true;
        Serial.println("frontWall= true;");
    } else {
        frontWall=false;
    }
    
    if (sensorValues[LEFT_FRONT] < WALL_THRESHOLD && sensorValues[LEFT_BACK] < WALL_THRESHOLD) {
        leftWall= true;
        Serial.println("leftWall= true;");
    } else {
        leftWall=false;
    }
    
    if (sensorValues[RIGHT_FRONT] < WALL_THRESHOLD && sensorValues[RIGHT_BACK] < WALL_THRESHOLD) {
        rightWall=true;
        Serial.println("rightWall= true;");
    } else {
        rightWall=false;
    }
    
    switch (state->orientation) {
        case NORTH:
            northWall = frontWall;
            eastWall = rightWall;
            westWall = leftWall;
            southWall = false;
            break;
        case SOUTH:
            southWall = frontWall;
            eastWall = leftWall;
            westWall = rightWall;
            northWall = false;
            break;
        case WEST:
            westWall = frontWall;
            northWall = rightWall;
            southWall = leftWall;
            eastWall = false;
            break;
        case EAST:
            eastWall = frontWall;
            northWall = leftWall;
            southWall = rightWall;
            westWall = false;
            break;
    }
    
    maze->addNode(state->pos->x, state->pos->y, northWall, southWall, eastWall, westWall);
}

void
moveLeft()
{
    go(LEFT, TURN);
    state->update(LEFT);
    delay(DELAY_DEBUG);
    go(FORWARD, SQUARE);
    state->update(FORWARD);
    delay(DELAY_DEBUG);
}

void moveRight()
{
    go(RIGHT, TURN);
    state->update(RIGHT);
    delay(DELAY_DEBUG);
    go(FORWARD, SQUARE);
    state->update(FORWARD);
    delay(DELAY_DEBUG);
}

void moveForward()
{
    go(FORWARD, SQUARE);
    state->update(FORWARD);
    delay(DELAY_DEBUG);
}

void moveBackward()
{
    go(RIGHT, TURN);
    state->update(RIGHT);
    delay(DELAY_DEBUG);
    go(RIGHT, TURN);
    state->update(RIGHT);
    delay(DELAY_DEBUG);
    go(FORWARD, SQUARE);
    state->update(FORWARD);
    delay(DELAY_DEBUG);
}

//for each iteration of the central loop, detectWalls is called, then nextMove

int
nextMove(struct coord start, struct coord finish)
{
    struct coord diff, **path;
    
    path = maze->shortestPath(start, finish);
    
    diff.x = path[0]->x - state->pos->x;
    diff.y = path[0]->y - state->pos->y;
    
    if(diff.x && diff.x > 0) {
        switch (state->orientation) {
            case NORTH:
                moveRight();
                break;
            case EAST:
                moveForward();
                break;
            case SOUTH:
                moveLeft();
                break;
            case WEST:
                moveBackward();
                break;
        }
    } else(diff.x && diff.x < 0) {
        switch (state->orientation) {
            case NORTH:
                moveLeft();
                break;
            case EAST:
                moveBackward();
                break;
            case SOUTH:
                moveRight();
                break;
            case WEST:
                moveForward();
                break;
        }
    } else(diff.y && diff.y > 0) {
        switch (state->orientation) {
            case NORTH:
                moveForward();
                break;
            case EAST:
                moveLeft();
                break;
            case SOUTH:
                moveBackward();
                break;
            case WEST:
                moveRight();
                break;
        }
    } else(diff.y && diff.y < 0) {
        switch (state->orientation) {
            case NORTH:
                moveBackward();
                break;
            case EAST:
                moveRight();
                break;
            case SOUTH:
                moveForward();
                break;
            case WEST:
                moveLeft();
                break;
        }
    }
    
    free(path);
}