// djikstra.ino

#include <stdio.h>

#define MAXNODES 16
#define ROW 4
#define COL 4
typedef struct node{
    int x;
    int y;
} node;

int grid[ROW][COL];
int costMatrix[MAXNODES][MAXNODES];
String pathMatrix[MAXNODES][MAXNODES];
int nodeMatrix[MAXNODES];
node unsettledNodes[36];
int unsettledNodeCounter = 0;
node settledNodes[36];
int settledNodeCounter = 0;
node startNode;
const int MAXVALUE = 9999;


void setup() {

}

void loop() {

}

