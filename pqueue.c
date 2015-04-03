#include "pqueue.h"

// implement a min heap

#define LEFT (0)
#define RIGHT (1)

// index of child
#define Child(x, dir) (2*(x) + (dir) + 1)

// index of parent
#define Parent(x) (((x)-1)/2)

#define INITIAL_SIZE (256)

typedef struct Node
{
    int walls;
    int distance;
} Node;

typedef struct priorityQueue
{
    Node *heap;
    int size;
} PQueue;

PQueue *pqueueCreate (int numNodes)
{

}