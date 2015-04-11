//
//  dijkstra.cpp
//  
//
//  Created by Henry Li on 4/2/15.
//

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <inttypes.h>
#define QUEUE_SIZE (300)
#define INFINITY (255)
#define MAZE_WIDTH (6)
#define MAZE_HEIGHT (6)
#define PATH_LENGTH (100)

// finds child 0 or child 1 of x (dir = 0 or 1, respectively)
#define Child(x, dir) (2*(x)+1+(dir))

//finds parent of x
#define Parent(x) (((x)-1)/2)

struct coord *path[PATH_LENGTH];

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
private:
    int size;
    int n;
    struct node *p[QUEUE_SIZE];
    
public:
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
    struct node nodes[MAZE_WIDTH][MAZE_HEIGHT];
    PriorityQueue *pqueue;
    
public:
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
    
    void
    shortestPath(struct coord start, struct coord finish) {
        int i, alt;
        struct coord neighbor, swap;
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
        
        while(pqueue->returnSize() > 0) {
            smallest = pqueue->dequeue();
            
            if(smallest->x == finish.x && smallest->y == finish.y) {
                for(i = 0; smallest->previous != 0; i++) {
                    struct coord pathpt;
                    pathpt.x = smallest->x;
                    pathpt.y = smallest->y;
                    path[i] = &pathpt;
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
                    
                    if(alt < this->nodes[neighbor.x][neighbor.y].distance) {
                        this->nodes[neighbor.x][neighbor.y].distance = alt;
                        this->nodes[neighbor.x][neighbor.y].previous = smallest;
                        pqueue->enqueue(&this->nodes[neighbor.x][neighbor.y]);
                    }
                }
            }
        }
        
        return;
    }
};

int main(int argc, char **argv) {
    struct coord start, finish;
    bool N, E, S, W;
    Graph *maze = new Graph();
    
    for(int j = 0; j < MAZE_WIDTH; j++) {
        for (int i = 0; i < MAZE_HEIGHT; i++) {
            N = S = E = W = 0;
            if(i == 0) W = true;
            if(j == 0) S = true;
            if(i == 5) E = true;
            if(j == 5) N = true;
            maze->addNode(i, j, N, E, S, W);
        }
    }
    
    start.x = 0;
    start.y = 0;
    finish.x = 5;
    finish.y = 5;
    
    maze->shortestPath(start, finish);
    
    for(int i = 0; path[i] != 0; i++) {
        printf("x: %d, y: %d\n", path[i]->x, path[i]->y);
    }
    
    delete maze;
    
    return 0;
}