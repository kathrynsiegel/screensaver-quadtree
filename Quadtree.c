/**
 * Quadtree.c -- Quadtree data structure to improve collision detections
 *
 * Function definitions in Quadtree.h
 **/
 
#include "Line.h"
#include "Vec.h"
#include "IntersectionEventList.h"

Quadtree* Quadtree_make(CollisionWorld* collisionWorld) {
  Quadtree* quadtree = malloc(sizeof(Quadtree));
  if (quadtree == NULL) {
    return NULL;
  }

  quadtree->collisionWorld = collisionWorld;
  quadtree->root = malloc(sizeof(QuadtreeNode*));
  
  return quadtree;
}

void Quadtree_delete(Quadtree* quadtree){
  QuadtreeNode_delete(quadtree->root);
  free(quadtree->root);
  free(quadtree);
}

// Recursively finds all collisions in this quadtree, adds them to the eventList, 
// and returns the number of collisions
unsigned int detectCollisions(Quadtree* quadtree, IntersectionEventList* intersectionEventList) {

}


