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
#include "IntersectionDetection.h"

Quadtree* Quadtree_new(CollisionWorld* collisionWorld, Vec upperLeft, Vec lowerRight) {
  Quadtree* quadtree = malloc(sizeof(Quadtree));
  if (quadtree == NULL) {
    return NULL;
  }

  quadtree->collisionWorld = collisionWorld;
  quadtree->upperLeft = upperLeft;
  quadtree->lowerRight = lowerRight;
  
  quadtree->numOfLines = 0;
  
  quadtree->lines = malloc(MAX_LINES_PER_NODE * sizeof(Line*));
  
  quadtree->isLeaf = !shouldDivideTree(quadtree);
  if (quadtree->isLeaf){
    
    //findLines(quadtree);
  } 
  else {
    quadtree->quadrants = malloc(4 * sizeof(Quadtree*));
    divideTree(quadtree);
    //printf("Done dividing\n");
  }
  //printf("%d\n", quadtree->numOfLines);
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
  for (int i = 0; i < quadtree->collisionWorld->numOfLines; i++){
    Line* line = quadtree->collisionWorld->lines[i];
    
    if (isLineInQuadtree(quadtree, line)){
      //printf("here");
      // printf("p1 x: %f y:%f\n",line->p1.x,line->p1.y);
//       printf("p2 x: %f y:%f\n",line->p2.x,line->p2.y);
//       printf("upperLeft x:%f y:%f\n",quadtree->upperLeft.x, quadtree->upperLeft.y);
//       printf("lowerRight x:%f y:%f\n",quadtree->lowerRight.x, quadtree->lowerRight.y);
      if (!addLine(quadtree, line)){
        return true;
      }
    }
  }
  return false;
}

void divideTree(Quadtree* quadtree){
  //printf("Dividing tree\n");
  Vec centerPoint = Vec_divide(Vec_add(quadtree->lowerRight,quadtree->upperLeft),2);;
  //printf("upperLeft x:%f y:%f\n",quadtree->upperLeft.x, quadtree->upperLeft.y);
  //printf("lowerRight x:%f y:%f\n",quadtree->lowerRight.x, quadtree->lowerRight.y);
  //printf("centerPoint x:%f y:%f\n",centerPoint.x, centerPoint.y);
  quadtree->quadrants[0] = Quadtree_new(quadtree->collisionWorld, 
    quadtree->upperLeft, 
    centerPoint);
  quadtree->quadrants[1] = Quadtree_new(quadtree->collisionWorld, 
    Vec_make(centerPoint.x, quadtree->upperLeft.y), 
    Vec_make(quadtree->lowerRight.x, centerPoint.y));
  quadtree->quadrants[2] = Quadtree_new(quadtree->collisionWorld, 
    Vec_make(quadtree->upperLeft.x, centerPoint.y), 
    Vec_make(centerPoint.x, quadtree->lowerRight.y));
  quadtree->quadrants[3] = Quadtree_new(quadtree->collisionWorld, 
    centerPoint, 
    quadtree->lowerRight);
}

void findLines(Quadtree* quadtree){}

bool addLine(Quadtree* quadtree, Line* line){
  quadtree->numOfLines++;
  if (quadtree->numOfLines > MAX_LINES_PER_NODE){
    return false;
  }
  quadtree->lines[quadtree->numOfLines-1] = line;
  return true;
}

bool isLineInQuadtree(Quadtree* quadtree, Line* line){
  // make the bounding box of the quadtree
  Vec box_p1 = quadtree->upperLeft;
  Vec box_p4 = quadtree->lowerRight;
  Vec box_p2 = Vec_make(box_p1.x, box_p4.y);
  Vec box_p3 = Vec_make(box_p4.x, box_p1.y);
  
  // make the parallelogram formed by the moving line
  Vec line_p1 = line->p1;
  Vec line_p2 = line->p2;
  Vec line_p3 = Vec_add(line_p1, Vec_multiply(line->velocity, quadtree->collisionWorld->timeStep));
  Vec line_p4 = Vec_add(line_p2, Vec_multiply(line->velocity, quadtree->collisionWorld->timeStep));
  
  // check each of the points in the parallelogram 
  // and each of the points in the bounding box
  // need to consider both (parallelogram > bounding box) and (bounding box > parallelogram)
  if (pointInParallelogram(line_p1, box_p1, box_p2, box_p3, box_p4)) {
    return true;
  }
  if (pointInParallelogram(line_p2, box_p1, box_p2, box_p3, box_p4)) {
    return true;
  }
  if (pointInParallelogram(line_p3, box_p1, box_p2, box_p3, box_p4)) {
    return true;
  }
  if (pointInParallelogram(line_p4, box_p1, box_p2, box_p3, box_p4)) {
    return true;
  }
  if (pointInParallelogram(box_p1, line_p1, line_p2, line_p3, line_p4)) {
    return true;
  }
  if (pointInParallelogram(box_p2, line_p1, line_p2, line_p3, line_p4)) {
    return true;
  }
  if (pointInParallelogram(box_p3, line_p1, line_p2, line_p3, line_p4)) {
    return true;
  }
  if (pointInParallelogram(box_p4, line_p1, line_p2, line_p3, line_p4)) {
    return true;
  }
  return false;
}

unsigned int detectCollisions(Quadtree* quadtree, IntersectionEventList* intersectionEventList){
  unsigned int numLineLineCollisions = 0;
  if (quadtree->isLeaf){
    // iterate through all lines in the quadtree and detect collisions
    for (int i = 0; i < quadtree->numOfLines; i++) {
    Line *l1 = quadtree->lines[i];

      for (int j = i + 1; j < quadtree->numOfLines; j++) {
        Line *l2 = quadtree->lines[j];

        // intersect expects compareLines(l1, l2) < 0 to be true.
        // Swap l1 and l2, if necessary.
        if (compareLines(l1, l2) >= 0) {
          Line *temp = l1;
          l1 = l2;
          l2 = temp;
        }

        IntersectionType intersectionType =
            intersect(l1, l2, quadtree->collisionWorld->timeStep);
        if (intersectionType != NO_INTERSECTION) {
          IntersectionEventList_appendNode(intersectionEventList, l1, l2,
                                           intersectionType);
          numLineLineCollisions++;
        }
      }
    }
  } 
  else {
    // add the collisions for all of the leaves of this quadtree
    for (int i = 0; i < 4; i++) {
      numLineLineCollisions += detectCollisions(quadtree->quadrants[i], intersectionEventList);
    }
  }
  return numLineLineCollisions; 
}


