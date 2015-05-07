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

// Pathfinding:
#define QUEUE_SIZE (100)
#define INFINITY (32767)
#define MAZE_WIDTH (5)
#define MAZE_HEIGHT (5)
#define PATH_LENGTH (15)

// finds child 0 or child 1 of x (dir = 0 or 1, respectively)
#define Child(x, dir) (2*(x)+1+(dir))

//finds parent of x
#define Parent(x) (((x)-1)/2)

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

// Pathfinding
struct coord **path2;

struct coord
{
    int x;
    int y;
};

/*struct pathstruct
{
    struct coord **path;
};*/

struct node
{
    int x;
    int y;
    bool direction[4]; // N, E, S, W in that order
    
    int distance;
    struct node *previous;
};

class PriorityQueue
{
public:
    int size;
    int n;
    struct node *p[QUEUE_SIZE];
    
    PriorityQueue()
    {
        size = QUEUE_SIZE;
        n = 0;
    }
    
    ~PriorityQueue()
    {
    }
    
    void
    enqueue(struct node *node)
    {
        this->p[this->n] = node;
        floatUp(this->n++);
    }
    
    struct node*
    dequeue(void)
    {
        struct node *x;
        if(this->n <= 0) return 0;
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
        Serial.println("");
        Serial.println("floatUp");
        struct node *x, **a;
        a = this->p;
        
        x = a[pos];
        
        while(1) {
          Serial.print("a[pos].x: ");
                Serial.println(a[pos]->x);
                Serial.print("a[pos].y: ");
                Serial.println(a[pos]->y);
                Serial.print("a[pos].distance: ");
                Serial.println(a[pos]->distance);
                Serial.print("a[Parent(pos)].x: ");
                Serial.println(a[Parent(pos)]->x);
                Serial.print("a[Parent(pos)].y: ");
                Serial.println(a[Parent(pos)]->y);
                Serial.print("a[Parent(pos)].distance: ");
                Serial.println(a[Parent(pos)]->distance);
            if(pos > 0 && a[Parent(pos)]->distance > x->distance) {
                Serial.println("SWITCHED");
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
        Serial.println("");
        Serial.println("floatDown");
        struct node *x, **a;
        int n = this->n;
        a = this->p;
        
        x = a[pos];
        
        while(1) {
          Serial.print("a[pos].x: ");
                  Serial.println(a[pos]->x);
                  Serial.print("a[pos].y: ");
                  Serial.println(a[pos]->y);
                  Serial.print("a[pos].distance: ");
                  Serial.println(a[pos]->distance);
                  Serial.print("a[Child(pos, 1)].x: ");
                  Serial.println(a[Child(pos, 1)]->x);
                  Serial.print("a[Child(pos, 1)].y: ");
                  Serial.println(a[Child(pos, 1)]->y);
                  Serial.print("a[Child(pos, 1)].distance: ");
                  Serial.println(a[Child(pos, 1)]->distance);
                  Serial.print("a[Child(pos, 0)].x: ");
                  Serial.println(a[Child(pos, 0)]->x);
                  Serial.print("a[Child(pos, 0)].y: ");
                  Serial.println(a[Child(pos, 0)]->y);
                  Serial.print("a[Child(pos, 0)].distance: ");
                  Serial.println(a[Child(pos, 0)]->distance);
            if(Child(pos, 1) < n && a[Child(pos, 1)]->distance < a[Child(pos, 0)]->distance) {
                if(a[Child(pos, 1)]->distance < x->distance) {
                   Serial.println("SWITCHED CHILD 1");
                    a[pos] = a[Child(pos, 1)];
                    pos = Child(pos, 1);
                } else {
                    break;
                }
            } else if(Child(pos, 0) < n && a[Child(pos, 0)]->distance < x->distance) {
                /*Serial.print("a[pos].x: ");
                Serial.println(a[pos]->x);
                Serial.print("a[pos].y: ");
                  Serial.println(a[pos]->y);
                  Serial.print("a[pos].distance: ");
                  Serial.println(a[pos]->distance);
                  Serial.print("a[Child(pos, 0)].x: ");
                  Serial.println(a[Child(pos, 0)]->x);
                  Serial.print("a[Child(pos, 0)].y: ");
                  Serial.println(a[Child(pos, 0)]->y);
                  Serial.print("a[Child(pos, 0)].distance: ");
                  Serial.println(a[Child(pos, 0)]->distance);*/
                  Serial.println("SWITCHED CHILD 0");
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
public:
    struct node nodes[MAZE_WIDTH][MAZE_HEIGHT];
    PriorityQueue *pqueue;
    
    Graph()
    {
        pqueue = new PriorityQueue();
    }
    
    ~Graph()
    {
        delete pqueue;
    }
    
    void
    addNode(int x, int y, bool north, bool east, bool south, bool west) {
        this->nodes[x][y].x = x;
        this->nodes[x][y].y = y;
        
        this->nodes[x][y].direction[0] = north;
        this->nodes[x][y].direction[1] = east;
        this->nodes[x][y].direction[2] = south;
        this->nodes[x][y].direction[3] = west;
    }
    
    struct coord **
    shortestPath(struct coord start, struct coord finish) {
        int i, alt;
        struct coord neighbor, swap, **path, *pathpt;
        struct node* smallest;
        
        for(int j = 0; j < MAZE_WIDTH; j++) {
            for(int i = 0; i < MAZE_HEIGHT; i++) {
                if(i == start.x && j == start.y) {
                    this->nodes[i][j].distance = 0;
                    this->nodes[i][j].previous = 0;
                    pqueue->enqueue(&this->nodes[i][j]);
                } else {
                    this->nodes[i][j].distance = INFINITY;
                    this->nodes[i][j].previous = 0;
                    pqueue->enqueue(&this->nodes[i][j]);
                }
            }
        }
        Serial.println("NEW");
        while(pqueue->returnSize() > 0) {
            Serial.print("p.x: ");
            Serial.println(pqueue->p[0]->x);
            Serial.print("p.y: ");
            Serial.println(pqueue->p[0]->y);
            Serial.print("p.distance: ");
            Serial.println(pqueue->p[0]->distance);
            smallest = pqueue->dequeue();
            Serial.print("Smallest.x: ");
            Serial.println(smallest->x);
            Serial.print("Smallest.y: ");
            Serial.println(smallest->y);
            Serial.print("Smallest.distance: ");
            Serial.println(smallest->distance);
            
            if(smallest->x == finish.x && smallest->y == finish.y) {
              path = (struct coord **) malloc(sizeof(*path)*PATH_LENGTH);
              Serial.println("PATH RETURN");
                  Serial.println("PATH RETURN");
                  Serial.println("PATH RETURN");
                  Serial.println("PATH RETURN");
                  Serial.println("PATH RETURN");
                  Serial.println("PATH RETURN");
                  Serial.println("PATH RETURN");
                  Serial.println("PATH RETURN");
                  Serial.println("PATH RETURN");
                  Serial.println("PATH RETURN");
                for(i = 0; smallest->previous != 0; i++) {
                  pathpt = (struct coord *) malloc(sizeof(*pathpt));
                  Serial.print("Smallest.x: ");
                  Serial.println(smallest->x);
            Serial.print("Smallest.y: ");
            Serial.println(smallest->y);
            Serial.print("Smallest.distance: ");
            Serial.println(smallest->distance);
                    pathpt->x = smallest->x;
                    pathpt->y = smallest->y;
                    path[i] = pathpt;
                    Serial.print("path[i].x: ");
                  Serial.println(path[i]->x);
            Serial.print("path[i].y: ");
            Serial.println(path[i]->y);
                    smallest = smallest->previous;
                }
                
                // reverse path array
                Serial.print("iiiiiii: ");
            Serial.println(i);
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
                    
                    if(alt < this->nodes[neighbor.x][neighbor.y].distance) {
                        this->nodes[neighbor.x][neighbor.y].distance = alt;
                        this->nodes[neighbor.x][neighbor.y].previous = smallest;
                        Serial.print("neighbor.x: ");
                        Serial.println(nodes[neighbor.x][neighbor.y].x);
                        Serial.print("neighbor.y: ");
                        Serial.println(nodes[neighbor.x][neighbor.y].y);
                        Serial.print("neighbor.distance: ");
                        Serial.println(nodes[neighbor.x][neighbor.y].distance);
                        pqueue->enqueue(&this->nodes[neighbor.x][neighbor.y]);
                    }
                }
            }
        }
        
        return path;
    }
};

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

// Pathfinding:
Graph *maze = new Graph();\
struct coord finish, start;
int No, So, Ea, We;

struct node *new2;

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
  
  Serial.println ("SETUP COMPLETE");
  delay(2000);
}


void loop() {
    for(int j = 0; j < MAZE_WIDTH; j++) {
        for (int i = 0; i < MAZE_HEIGHT; i++) {
            No = So = Ea = We = 0;
            if(i == 0) We = true;
            if(j == 0) So = true;
            if(i == 4) Ea = true;
            if(j == 4) No = true;
            maze->addNode(i, j, No, Ea, So, We);
        }
    }
    
    start.x = 0;
    start.y = 0;
    finish.x = 4;
    finish.y = 4;
    
    path2 = maze->shortestPath(start, finish);
        
    for(int i = 0; path2[i] != 0; i++) {
        Serial.print("x: ");
        Serial.println((*path2[i]).x);
        Serial.print("y: ");
        Serial.println((*path2[i]).y);
        free(path2[i]);
    }
    free(path2);
    
    Serial.println("NEW LOOP()");
    Serial.println("NEW LOOP()");
    Serial.println("NEW LOOP()");
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


// simple algorithm
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
