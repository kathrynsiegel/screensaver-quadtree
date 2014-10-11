/**
 * Quadtree.c -- Quadtree data structure to improve collision detections
 *
 * Function definitions in Quadtree.h
 **/
 
#include "Line.h"
#include "Vec.h"
#include "IntersectionEventList.h"

Quadtree* Quadtree_new(Vec* upperLeft, Vec* lowerRight) {
  Quadtree* quadtree = malloc(sizeof(Quadtree));
  if (quadtree == NULL) {
    return NULL;
  }

  quadtree->upperLeft = upperLeft;
  quadtree->lowerRight = lowerRight;
  quadtree->lines = malloc(MAX_LINES_PER_NODE * sizeof(Line*));
  quadtree->quadrants = malloc(4 * sizeof(Quadtree*));
  quadtree->numOfLines = 0;
  
  quadtree->isLeaf = !shouldDivideTree(quadtree);
  if (!(quadtree->isLeaf)){
    divideTree(quadtree);
  }
  findLines(quadtree);
  
  return quadtree;
}

void Quadtree_delete(Quadtree* quadtree){
  if (!(quadtree->isLeaf)){
    for (int i = 0; i < 4; i++) {
      Quadtree_delete(quadtree->quadrants[i]);
    }
  }
  free(quadtree->lines);
  free(quadtree->quadrants);
  free(quadtree);
}

bool shouldDivideTree(Quadtree* quadtree){}

void divideTree(Quadtree* quadtree, Quadtree** quadrants){}

void findLines(Quadtree* quadtree){}

void addLine(Quadtree* quadtree, Line* line){}

unsigned int detectCollisions(Quadtree* quadtree, IntersectionEventList* intersectionEventList){}


