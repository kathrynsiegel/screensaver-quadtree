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
#include <cilk/cilk.h>
#include <cilk/reducer.h>
#include <cilk/reducer_opadd.h>

///////////////////////////////////////////////////////////
// Create the quadtree.
//
// collisionWorld -> the collision world in which the quadtree is created
// upperLeft -> the upper left point of the quadtree
// lowerRight -> the lower right point of the quadtree
// parent -> the parent quadtree node of this quadtree node
Quadtree* Quadtree_new(CollisionWorld* collisionWorld, Vec upperLeft, Vec lowerRight, Quadtree* parent) {
  Quadtree* quadtree = malloc(sizeof(Quadtree));
  if (quadtree == NULL) {
    return NULL;
  }
  quadtree->collisionWorld = collisionWorld;
  quadtree->upperLeft = upperLeft;
  quadtree->lowerRight = lowerRight;

  // set the parent
  if (parent != NULL){
    quadtree->parent = parent;
    quadtree->depth = quadtree->parent->depth+1;
  } else {
    quadtree->parent = NULL;
    quadtree->depth = 0;
  }

  // allocate the line array
  quadtree->numOfLines = 0;
  quadtree->lines = malloc(MAX_LINES_PER_NODE * sizeof(Line*));
  quadtree->isLeaf = !shouldDivideTree(quadtree);

  // if the quadtree is not a leaf, then recursively create four
  // new quadtree nodes
  if (!(quadtree->isLeaf)){
    quadtree->quadrants = malloc(4 * sizeof(Quadtree*));
    divideTree(quadtree);
  } else {
    updateLines(quadtree);
  }
  return quadtree;
}

///////////////////////////////////////////////////////////
// Delete the quadtree and deallocate.
// We parallelize the deletion
void Quadtree_delete(Quadtree* quadtree){
  free(quadtree->lines);
  if (!(quadtree->isLeaf)){
    cilk_spawn Quadtree_delete(quadtree->quadrants[0]);
    cilk_spawn Quadtree_delete(quadtree->quadrants[1]);
    cilk_spawn Quadtree_delete(quadtree->quadrants[2]);
    Quadtree_delete(quadtree->quadrants[3]);
    cilk_sync;
    free(quadtree->quadrants);
  }
  free(quadtree);
}

///////////////////////////////////////////////////////////
// Update the quadtree--we parallelize this update.
void Quadtree_update(Quadtree* quadtree){
  if (quadtree->isLeaf){
    updateLines(quadtree);
  }
  else {
    cilk_for (int i = 0; i < 4; i++) {
       Quadtree_update(quadtree->quadrants[i]);
    }
  }
}

///////////////////////////////////////////////////////////
// Update all of the line positions of the lines in the quadtree.
inline void updateLines(Quadtree* quadtree){
  quadtree->numOfLines = 0;
  int qNum = quadtree->collisionWorld->numOfLines;
  for (int i = 0; i < qNum; i++){
    Line* line = quadtree->collisionWorld->lines[i];
    if (isLineInQuadtree(quadtree, line)){
      addLine(quadtree, line);
    }
  }
}

///////////////////////////////////////////////////////////
// Determine whether we should divide the quadtree into four
// separate quadtree nodes.
inline bool shouldDivideTree(Quadtree* quadtree){
  return quadtree->depth < MAX_DEPTH;
}

///////////////////////////////////////////////////////////
// Divides the given quadtree into four separate quadtree nodes.
inline void divideTree(Quadtree* quadtree){
  // break the tree up into 4 quadrants
  Vec centerPoint = Vec_divide(Vec_add(quadtree->lowerRight,quadtree->upperLeft),2);
  quadtree->quadrants[0] = Quadtree_new(quadtree->collisionWorld, 
    quadtree->upperLeft, 
    centerPoint,
    quadtree);
  quadtree->quadrants[1] = Quadtree_new(quadtree->collisionWorld, 
    Vec_make(centerPoint.x, quadtree->upperLeft.y), 
    Vec_make(quadtree->lowerRight.x, centerPoint.y),
    quadtree);
  quadtree->quadrants[2] = Quadtree_new(quadtree->collisionWorld, 
    Vec_make(quadtree->upperLeft.x, centerPoint.y), 
    Vec_make(centerPoint.x, quadtree->lowerRight.y),
    quadtree);
  quadtree->quadrants[3] = Quadtree_new(quadtree->collisionWorld, 
    centerPoint, 
    quadtree->lowerRight,
    quadtree);
}

///////////////////////////////////////////////////////////
// Adds a line to a given quadtree.
inline bool addLine(Quadtree* quadtree, Line* line){
  quadtree->numOfLines++;
  if (quadtree->numOfLines > MAX_LINES_PER_NODE){
    quadtree->numOfLines = 0;
    return false;
  }
  quadtree->lines[quadtree->numOfLines-1] = line;
  return true;
}

///////////////////////////////////////////////////////////
// Checks whether a line is in the quadtree. 
// We treat the line as a parallelogram and the quadtree
// as a box, and check whether the parallelogram and the quadtree
// box overlap.
inline bool isLineInQuadtree(Quadtree* quadtree, Line* line){
  // make the half of the bounding box of the quadtree
  Vec box_p1 = quadtree->upperLeft;
  Vec box_p4 = quadtree->lowerRight;
  
  // make the parallelogram formed by the moving line
  Vec line_p1 = line->p1;
  Vec line_p2 = line->p2;
  Vec line_p3 = line->p3;
  Vec line_p4 = line->p4;
  
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
 
  // check each of the points in the parallelogram 
  // and each of the points in the bounding box
  // need to consider both (parallelogram > bounding box) and (bounding box > parallelogram)
  if (pointInSquare(line_p1, box_p1, box_p4)) {
    return true;
  }
  if (pointInSquare(line_p2, box_p1, box_p4)) {
    return true;
  }
  if (pointInSquare(line_p3, box_p1, box_p4)) {
    return true;
  }
  if (pointInSquare(line_p4, box_p1, box_p4)) {
    return true;
  }
  
  // finish the bounding box once we know we need to check all corners
  Vec box_p3 = Vec_make(box_p1.x, box_p4.y);
  Vec box_p2 = Vec_make(box_p4.x, box_p1.y);
  
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

  return false;
}

///////////////////////////////////////////////////////////
// Use reducers to detect whether
void detectCollisionsReducer(Quadtree* quadtree, IntersectionEventListReducer* intersectionEventList, CILK_C_REDUCER_OPADD_TYPE(int)* numCollisions){
  if (quadtree->isLeaf){
    // iterate through all lines in the quadtree and detect collisions
    //double timestep = quadtree->collisionWorld->timeStep;
    
    cilk_for (int i = 0; i < quadtree->numOfLines; i++) {
      Line *l1 = quadtree->lines[i];


      for (int j = i+1; j < quadtree->numOfLines; j++) {
        Line *l2 = quadtree->lines[j];

        // intersect expects compareLines(l1, l2) < 0 to be true.
        // Swap l1 and l2, if necessary.
        if (compareLines(l1, l2) >= 0) {
          Vec p1;
          Vec p2;
          // Get relative velocity.
          Vec shift;
          shift.x = l1->shift.x - l2->shift.x;
          shift.y = l1->shift.y - l2->shift.y;

          // Get the parallelogram.
          p1.x = l1->p1.x + shift.x;
          p1.y = l1->p1.y + shift.y;
  
          p2.x = l1->p2.x + shift.x;
          p2.y = l1->p2.y + shift.y;
          if (fastIntersect(l2, l1, p1, p2)) {
            IntersectionEventList_appendNode(&REDUCER_VIEW(*intersectionEventList), l2, l1,
                                    intersect(l2, l1, p1, p2));
            REDUCER_VIEW(*numCollisions)++;        
          }
        }
        else {
          Vec p1;
          Vec p2;
          // Get relative velocity.
          Vec shift;
          shift.x = l2->shift.x - l1->shift.x;
          shift.y = l2->shift.y - l1->shift.y;

          // Get the parallelogram.
          p1.x = l2->p1.x + shift.x;
          p1.y = l2->p1.y + shift.y;
  
          p2.x = l2->p2.x + shift.x;
          p2.y = l2->p2.y + shift.y;
          if (fastIntersect(l1, l2, p1, p2)) {
            IntersectionEventList_appendNode(&REDUCER_VIEW(*intersectionEventList), l1, l2,
                                    intersect(l1, l2, p1, p2));
            REDUCER_VIEW(*numCollisions)++;        
          }
        }
      }
    }
  } 
  else {
    // add the collisions for all of the leaves of this quadtree
    cilk_for (int i = 0; i < 4; i++) {
      detectCollisionsReducer(quadtree->quadrants[i], intersectionEventList, numCollisions);
    }
  }
}


