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

inline Quadtree* Quadtree_new(CollisionWorld* collisionWorld, Vec upperLeft, Vec lowerRight) {
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
  if (!(quadtree->isLeaf)){
    quadtree->quadrants = malloc(4 * sizeof(Quadtree*));
    divideTree(quadtree);
  }
  
  return quadtree;
}

inline void Quadtree_delete(Quadtree* quadtree){
  free(quadtree->lines);
  if (!(quadtree->isLeaf)){
    for (int i = 0; i < 4; i++) {
      Quadtree_delete(quadtree->quadrants[i]);
    }
    free(quadtree->quadrants);
  }
  free(quadtree);
}

inline bool Quadtree_update(Quadtree* quadtree){
  bool shouldDestroy = false;
  if (quadtree->isLeaf){
    shouldDestroy = shouldDivideTree(quadtree);
  }
  else {
    // add the collisions for all of the leaves of this quadtree
    unsigned int numLinesUnder = 0;
    for (int i = 0; i < 4; i++) {
       shouldDestroy = Quadtree_update(quadtree->quadrants[i]);
       if (shouldDestroy) {
         Vec upperLeft = quadtree->quadrants[i]->upperLeft;
         Vec lowerRight = quadtree->quadrants[i]->lowerRight;
         Quadtree_delete(quadtree->quadrants[i]);
         quadtree->quadrants[i] = Quadtree_new(quadtree->collisionWorld, 
           upperLeft, 
           lowerRight);
       }
       numLinesUnder += getNumLinesUnder(quadtree->quadrants[i]);
    }
    if (numLinesUnder <= MAX_LINES_PER_NODE) {
      shouldDestroy = true;
    }
  }
  return shouldDestroy;

}

inline bool shouldDivideTree(Quadtree* quadtree){
  quadtree->numOfLines = 0;
  for (int i = 0; i < quadtree->collisionWorld->numOfLines; i++){
    Line* line = quadtree->collisionWorld->lines[i];
    
    if (isLineInQuadtree(quadtree, line)){
      if (!addLine(quadtree, line)){
        // if the line did not add, then there are too many lines, so divide the quadtree 
        return true;
      }
    }
  }
  return false;
}

inline void divideTree(Quadtree* quadtree){
  // break the tree up into 4 quadrants
  Vec centerPoint = Vec_divide(Vec_add(quadtree->lowerRight,quadtree->upperLeft),2);;
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

inline unsigned int getNumLinesUnder(Quadtree* quadtree){
  unsigned int numLinesUnder = 0;
  if (quadtree->isLeaf) {
    return quadtree->numOfLines;
  }
  for (int i = 0; i < 4; i++) {
    numLinesUnder += getNumLinesUnder(quadtree->quadrants[i]);
  }
  return numLinesUnder;
}

inline bool addLine(Quadtree* quadtree, Line* line){
  quadtree->numOfLines++;
  if (quadtree->numOfLines > MAX_LINES_PER_NODE){
    return false;
  }
  quadtree->lines[quadtree->numOfLines-1] = line;
  return true;
}

inline bool isLineInQuadtree(Quadtree* quadtree, Line* line){
  // make the half of the bounding box of the quadtree
  Vec box_p1 = quadtree->upperLeft;
  Vec box_p4 = quadtree->lowerRight;
  
  // make the parallelogram formed by the moving line
  Vec line_p1 = line->p1;
  Vec line_p2 = line->p2;
  Vec line_p3 = Vec_add(line_p1, Vec_multiply(line->velocity, quadtree->collisionWorld->timeStep));
  Vec line_p4 = Vec_add(line_p2, Vec_multiply(line->velocity, quadtree->collisionWorld->timeStep));
  
  // perform a preliminary check if all of the parallelogram points are off to a side of the box
  if (line_p1.x > box_p4.x && line_p2.x > box_p4.x && line_p3.x > box_p4.x && line_p4.x > box_p4.x){
    return false;
  }
  if (line_p1.y < box_p1.y && line_p2.y < box_p1.y && line_p3.y < box_p1.y && line_p4.x < box_p1.y){
    return false;
  }
  if (line_p1.y > box_p4.y && line_p2.y > box_p4.y && line_p3.y > box_p4.y && line_p4.y > box_p4.y){
    return false;
  }
  if (line_p1.x < box_p1.x && line_p2.x < box_p1.x && line_p3.x < box_p1.x && line_p4.x < box_p1.x){
    return false;
  }
  
  // finish the bounding box once we know we need to check all corners
  Vec box_p3 = Vec_make(box_p1.x, box_p4.y);
  Vec box_p2 = Vec_make(box_p4.x, box_p1.y);
  
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
  
  //also check all the lines in the case that no points are inside the other shape
  if (intersectLines(box_p1, box_p2, line_p1, line_p2)){
    return true;
  }
  if (intersectLines(box_p1, box_p3, line_p1, line_p2)){
    return true;
  }
  if (intersectLines(box_p2, box_p4, line_p1, line_p2)){
    return true;
  }
  if (intersectLines(box_p3, box_p4, line_p1, line_p2)){
    return true;
  }
  
  if (intersectLines(box_p1, box_p2, line_p1, line_p3)){
    return true;
  }
  if (intersectLines(box_p1, box_p3, line_p1, line_p3)){
    return true;
  }
  if (intersectLines(box_p2, box_p4, line_p1, line_p3)){
    return true;
  }
  if (intersectLines(box_p3, box_p4, line_p1, line_p3)){
    return true;
  }
  
//   if (intersectLines(box_p1, box_p2, line_p2, line_p4)){
//     return true;
//   }
//   if (intersectLines(box_p1, box_p3, line_p2, line_p4)){
//     return true;
//   }
//   if (intersectLines(box_p2, box_p4, line_p2, line_p4)){
//     return true;
//   }
//   if (intersectLines(box_p3, box_p4, line_p2, line_p4)){
//     return true;
//   }
  
//   if (intersectLines(box_p1, box_p2, line_p3, line_p4)){
//     return true;
//   }
//   if (intersectLines(box_p1, box_p3, line_p3, line_p4)){
//     return true;
//   }
//   if (intersectLines(box_p2, box_p4, line_p3, line_p4)){
//     return true;
//   }
//   if (intersectLines(box_p3, box_p4, line_p3, line_p4)){
//     return true;
//   }
  return false;
}

// Bentley Ottman
// inline unsigned int fastDetectCollisions(Quadtree* quadtree, IntersectionEventList* intersectionEventList){

// }

inline unsigned int detectCollisions(Quadtree* quadtree, IntersectionEventList* intersectionEventList){
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

        // IntersectionType intersectionType =
        //     intersect(l1, l2, quadtree->collisionWorld->timeStep);
        if (fastIntersect(l1, l2, quadtree->collisionWorld->timeStep)) {
          
          
          IntersectionEventNode* newNode = malloc(sizeof(IntersectionEventNode));
          newNode->l1 = l1;
          newNode->l2 = l2;
          
          // check to make sure this collision has not been added already
          
          bool alreadyAdded = false;
          IntersectionEventNode* curNode = intersectionEventList->head;
          IntersectionEventNode* nextNode = NULL;
          
          while (curNode != NULL) {
            nextNode = curNode->next;
            if (IntersectionEventNode_compareData(newNode, curNode) == 0){
              alreadyAdded=true;
            }
            curNode = nextNode;
          }
          
          free(newNode);
          
          if (!alreadyAdded){
            IntersectionEventList_appendNode(intersectionEventList, l1, l2,
                                    intersect(l1, l2, quadtree->collisionWorld->timeStep));
            numLineLineCollisions++;
          }
          
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


