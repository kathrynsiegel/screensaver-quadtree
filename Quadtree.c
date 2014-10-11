/**
 * Quadtree.c -- Quadtree data structure to improve collision detections
 *
 * Function definitions in Quadtree.h
 **/
 
#include "Line.h"
#include "Vec.h"
#include "IntersectionEventList.h"
#include "QuadtreeNode.h"

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

void populateQuadtree(Quadtree* quadtree) {
  unsigned int numLines = quadtree->collisionWorld->numOfLines;
  Line** lines = quadtree->collisionWorld->lines;
  Line** quadtreeLines = malloc(numLines * sizeof(Line*));
  for (int i = 0; i < numLines; i++) {
    quadtreeLines[i] = lines[i];
  }

  Vec* upperLeft = malloc(sizeof(Vec*));
  upperLeft->x = BOX_XMIN;
  upperLeft->y = BOX_YMIN;
  Vec* lowerRight = malloc(sizeof(Vec*));
  lowerRight->x = BOX_XMAX;
  lowerRight->y = BOX_YMAX;

  quadtree->root = QuadtreeNode_make(quadtree->collisionWorld, upperLeft, lowerRight, quadtreeLines, numLines);
  if (numLines > MAX_LINES_PER_NODE) {
    populateQuadtreeNode(quadtree->root);
  }
}

// Recursively finds all collisions in this quadtree, adds them to the eventList, 
// and returns the number of collisions
unsigned int detectCollisions(Quadtree* quadtree, IntersectionEventList* intersectionEventList) {
  return detectNodeCollisions(quadtree->root);
}


