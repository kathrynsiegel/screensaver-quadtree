/**
 * Quadtree.c -- Quadtree data structure to improve collision detections
 *
 * Function definitions in Quadtree.h
 **/
 
#include "Quadtree.h"

#include <stdlib.h>
#include <stdio.h>

#include "CollisionWorld.h"
#include "Line.h"
#include "Vec.h"
#include "IntersectionEventList.h"

Quadtree* Quadtree_new(CollisionWorld* collisionWorld, Vec* upperLeft, Vec* lowerRight) {
  Quadtree* quadtree = malloc(sizeof(Quadtree));
  if (quadtree == NULL) {
    return NULL;
  }

  quadtree->collisionWorld = collisionWorld;
  quadtree->upperLeft = upperLeft;
  quadtree->lowerRight = lowerRight;
  quadtree->numOfLines = 0;
  
  quadtree->isLeaf = !shouldDivideTree(quadtree);
  if (quadtree->isLeaf){
    quadtree->lines = malloc(MAX_LINES_PER_NODE * sizeof(Line*));
    findLines(quadtree);
  } else {
    quadtree->quadrants = malloc(4 * sizeof(Quadtree*));
    divideTree(quadtree);
  }
  
  return quadtree;
}

void Quadtree_delete(Quadtree* quadtree){
  if (quadtree->isLeaf){
     free(quadtree->lines);
  } else {
    for (int i = 0; i < 4; i++) {
      Quadtree_delete(quadtree->quadrants[i]);
    }
    free(quadtree->quadrants);
  }
  free(quadtree);
}

bool shouldDivideTree(Quadtree* quadtree){
bool r = false;
return r;
}

void divideTree(Quadtree* quadtree){}

void findLines(Quadtree* quadtree){}

void addLine(Quadtree* quadtree, Line* line){}

unsigned int detectCollisions(Quadtree* quadtree, IntersectionEventList* intersectionEventList){
unsigned int r = 0;
return r;
}


