/**
 * Quadtree.h -- Quadtree data structure to improve collision detections
 *
 **/

#include "Line.h"
#include "Vec.h"
#include "IntersectionEventList.h"
#include "QuadtreeNode.h"

#define MAX_LINES_PER_NODE 10

struct Quadtree {
  // The CollisionWorld the Quadtree exists in
  CollisionWorld* collisionWorld;

  // The root of the tree
  QuadtreeNode* root;
};

typedef struct Quadtree Quadtree;

Quadtree* Quadtree_make(CollisionWorld* collisionWorld);

void Quadtree_delete(Quadtree* quadtree);

// populates the quadtree recursively starting at the root
void populateQuadtree(Quadtree* quadtree);

// Recursively finds all collisions in this quadtree, adds them to the eventList, 
// and returns the number of collisions
unsigned int detectCollisions(Quadtree* quadtree, IntersectionEventList* intersectionEventList);

