#include <stdint.h>

// Priority Queue

typedef struct priorityQueue PQueue;

typedef struct node Node;

PQueue *pqueueCreate (int numNodes);

void addWithPriority (Pqueue q, Node n, int distance);

void decreasePriority (Pqueue q, Node n, int distance);

void getMin (Pqueue q);