/**
 * Quadtree.h -- Quadtree data structure to improve collision detections
 *
 **/

#ifndef QUADTREE_H_
#define QUADTREE_H_

#include "CollisionWorld.h"
#include "Line.h"
#include "Vec.h"
#include "IntersectionEventList.h"

#include <cilk/reducer_opadd.h>

#define MAX_LINES_PER_NODE 155

// need to forward reference due to circularity of these structs
typedef struct CollisionWorld CollisionWorld;
typedef struct Quadtree Quadtree;

typedef struct Quadtree {

  // The CollisionWorld the Quadtree exists in
  CollisionWorld* collisionWorld;
  
  // The upper left corner of the Quadtree node
  Vec upperLeft;
  
  // The lower right corner of the Quadtree node
  Vec lowerRight;

  // Array containing all of the lines that are part of this level of the Quadtree
  // This array is only comprehensive if the node is a leaf
  Line** lines;
  unsigned int numOfLines;

  // Array containing four quadrants of this Quadtree
  Quadtree** quadrants;
  
  // True if the Quadtree contains less than MAX_LINES_PER_NODE lines
  bool isLeaf;
} Quadtree_t;



Quadtree* Quadtree_new(CollisionWorld* collisionWorld, Vec upperLeft, Vec lowerRight);

void Quadtree_delete(Quadtree* quadtree);

void Quadtree_update(Quadtree* quadtree);

// Returns true if this tree needs to divide itself into quadrants
// and adds all lines in this quadtree
bool shouldDivideTree(Quadtree* quadtree);

// Instantiates and fills the four quadrants of the tree
void divideTree(Quadtree* quadtree);

// Finds all of the lines that should belong to this quadtree level and adds them
void findLines(Quadtree* quadtree);

// Adds the line to the quadtree structure
bool addLine(Quadtree* quadtree, Line* line);

// Checks if the moving line is in the quadtree
bool isLineInQuadtree(Quadtree* quadtree, Line* line);

// Recursively finds all collisions in this quadtree, adds them to the eventList, 
// and returns the number of collisions
unsigned int detectCollisions(Quadtree* quadtree, IntersectionEventList* intersectionEventList);

void detectCollisionsReducer(Quadtree* quadtree, IntersectionEventListReducer* intersectionEventList, CILK_C_REDUCER_OPADD_TYPE(int)* numCollisions);

#endif  // QUADTREE_H_
