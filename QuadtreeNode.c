/**
 * QuadtreeNode.c -- QuadtreeNode data structure to improve collision detections
 *
 * Function definitions in QuadtreeNode.h
 **/

#include <stdlib.h>
#include <stdio.h>

#include "Line.h"
#include "Vec.h"
#include "IntersectionEventList.h"
#include "QuadtreeNode.h"

QuadtreeNode* QuadtreeNode_make(CollisionWorld* collisionWorld, Vec* upperLeft, Vec* lowerRight, Line** lines, unsigned int numLines) {
  QuadtreeNode* quadtreeNode = malloc(sizeof(QuadtreeNode));
  if (quadtreeNode == NULL) {
    return NULL;
  }

  quadtreeNode->collisionWorld = collisionWorld;
  quadtreeNode->upperLeft = upperLeft;
  quadtreeNode->lowerRight = lowerRight;
  quadtreeNode->lines = lines;
  quadtreeNode->quadrants = malloc(sizeof(QuadtreeNode) * 4);
  quadtreeNode->numOfLines = numLines;
  quadtreeNode->isLeaf = (numLines <= MAX_LINES_PER_NODE);
  
  return quadtreeNode;
}  

void QuadtreeNode_delete(QuadtreeNode* quadtreeNode) {
  for (int i = 0; i < 4; i++) {
    QuadtreeNode_delete(quadtreeNode->quadrants[i]);
  }
  free(quadtreeNode->upperLeft);
  free(quadtreeNode->lowerRight);
  free(quadtreeNode->lines);
}

// only called when we know that this node has to be split into quadrants
void populateQuadtreeNode(QuadtreeNode* quadtreeNode) {
  // count lines in each partition
  unsigned int numLinesUpperLeft = 0;
  unsigned int numLinesUpperRight = 0;
  unsigned int numLinesLowerLeft = 0;
  unsigned int numLinesLowerRight = 0;
  Vec* upperLeft = quadtreeNode->upperLeft;
  Vec* lowerRight = quadtreeNode->lowerRight;
  double xLowerBound = upperLeft->x;
  double xUpperBound = lowerRight->x;
  double yLowerBound = upperLeft->y;
  double yUpperBound = lowerRight->y;
  double yMidBound = (yLowerBound + yUpperBound)/2.0;
  double xMidBound = (xLowerBound + xUpperBound)/2.0;
  Vec midPoint = Vec_make(xMidBound,yMidBound);
  Vec lowerMid = Vec_make(xMidBound,yLowerBound);
  Vec rightMid = Vec_make(xUpperBound,yMidBound);
  Vec leftMid = Vec_make(xLowerBound,yMidBound);
  Vec bottomMid = Vec_make(xMidBound,yUpperBound);

  for (int i = 0; i < quadtreeNode->numOfLines; i++) {
    Line* line = quadtreeNode->lines[i];
    Line* movedLine = malloc(sizeof(Line*));
    movedLine->p1 = Vec_add(line->p1, line->velocity);
    movedLine->p2 = Vec_add(line->p2, line->velocity);

    // test if in upper left
    if (testInBox(xLowerBound,xMidBound,yLowerBound,yMidBound,line,movedLine)) {
      numLinesUpperLeft++;
    }

    // test if in upper right
    if (testInBox(xMidBound,xUpperBound,yLowerBound,yMidBound,line,movedLine)) {
      numLinesUpperRight++;
    }

    // test if in lower left
    if (testInBox(xLowerBound,xMidBound,yMidBound,yUpperBound,line,movedLine)) {
      numLinesLowerLeft++;
    }

    // test if in lower right
    if (testInBox(xMidBound,xUpperBound,yMidBound,yUpperBound,line,movedLine)) {
      numLinesLowerRight++;
    }
  }

  // create four Line* lists with the four partitions
  Line** linesUpperLeft = malloc(numLinesUpperLeft * sizeof(Line*));
  Line** linesUpperRight = malloc(numLinesUpperRight * sizeof(Line*));
  Line** linesLowerLeft = malloc(numLinesLowerLeft * sizeof(Line*));
  Line** linesLowerRight = malloc(numLinesLowerRight * sizeof(Line*));
  unsigned int linesUpperLeftIndex = 0;
  unsigned int linesUpperRightIndex = 0;
  unsigned int linesLowerLeftIndex = 0;
  unsigned int linesLowerRightIndex = 0;

  for (int i = 0; i < quadtreeNode->numOfLines; i++) {
    Line* line = quadtreeNode->lines[i];

    // test if in upper left
    if (testInBox(xLowerBound,xMidBound,yLowerBound,yMidBound,line,movedLine)) {
      linesUpperLeft[linesUpperLeftIndex] = line;
      linesUpperLeftIndex++;
    }

    // test if in upper right
    if (testInBox(xMidBound,xUpperBound,yLowerBound,yMidBound,line,movedLine)) {
      linesUpperRight[linesUpperRightIndex] = line;
      linesUpperRightIndex++;
    }

    // test if in lower left
    if (testInBox(xLowerBound,xMidBound,yMidBound,yUpperBound,line,movedLine)) {
      linesLowerLeft[linesLowerLeftIndex] = line;
      linesLowerLeftIndex++;
    }

    // test if in lower right
    if (testInBox(xMidBound,xUpperBound,yMidBound,yUpperBound,line,movedLine)) {
      linesLowerRight[linesLowerRightIndex] = line;
      linesLowerRightIndex++;
    }
  }

  // instantiate four quadtreeNodes
  quadtreeNode->quadrants[0] = QuadtreeNode_make(quadtreeNode->collisionWorld, upperLeft, &midPoint, linesUpperLeft, numLinesUpperLeft);
  if (numLinesUpperLeft > MAX_LINES_PER_NODE) {
    populateQuadtreeNode(quadtreeNode->quadrants[0]);
  }
  quadtreeNode->quadrants[1] = QuadtreeNode_make(quadtreeNode->collisionWorld, &lowerMid, &rightMid, linesUpperRight, numLinesUpperRight);
  if (numLinesUpperRight > MAX_LINES_PER_NODE) {
    populateQuadtreeNode(quadtreeNode->quadrants[1]);
  }
  quadtreeNode->quadrants[2] = QuadtreeNode_make(quadtreeNode->collisionWorld, &leftMid, &bottomMid, linesLowerLeft, numLinesLowerLeft);
  if (numLinesLowerLeft > MAX_LINES_PER_NODE) {
    populateQuadtreeNode(quadtreeNode->quadrants[2]);
  }
  quadtreeNode->quadrants[3] = QuadtreeNode_make(quadtreeNode->collisionWorld, &midPoint, lowerRight, linesLowerRight, numLinesLowerRight);
  if (numLinesLowerRight > MAX_LINES_PER_NODE) {
    populateQuadtreeNode(quadtreeNode->quadrants[3]);
  }
}

bool testInBox(double xLower, double xUpper, double yLower, double yUpper, Line* line, Line* movedLine) {
  if (testLineInBox(double xLower, double xUpper, double yLower, double yUpper, Line* line)) {
    return true;
  }
  if (testLineInBox(double xLower, double xUpper, double yLower, double yUpper, Line* movedLine)) {
    return true;
  }
  if (testLinesStraddleBox(double xLower, double xUpper, double yLower, double yUpper, Line* line, Line* movedLine)) {
    return true;
  }
  return false;
}

bool testLinesStraddleBox(double xLower, double xUpper, double yLower, double yUpper, Line* line, Line* movedLine) {
  
}

bool testLineInBox(double xLower, double xUpper, double yLower, double yUpper, Line* line) {
  Vec p1 = line->p1;
  Vec p2 = line->p2;
  double p1x = p1.x;
  double p2x = p2.x;
  double p1y = p1.y;
  double p2y = p2.y;

  // test if one point is in box
  if (p1x > xLower && p1x <= xUpper && p1y > yLower && p1y <= yUpper) {
    return true;
  }

  // test if other point is in box
  if (p2x > xLower && p2x <= xUpper && p2y > yLower && p2y <= yUpper) {
    return true;
  }

  // test if line overlaps box

  // If (x1 > xTR and x2 > xTR), no intersection (line is to right of rectangle). 
  // If (x1 < xBL and x2 < xBL), no intersection (line is to left of rectangle). 
  // If (y1 > yTR and y2 > yTR), no intersection (line is above rectangle). 
  // If (y1 < yBL and y2 < yBL), no intersection (line is below rectangle).
  if ((p1x > xUpper && p2x > xUpper) || (p1x < xLower && p2x < xLower) || (p1y > yUpper && p2y > yUpper) || (p1y < yLower && p2y < yLower)) {
    return false;
  }

  // F(x y) = (y2-y1)x + (x1-x2)y + (x2*y1-x1*y2)
  // If F(x y) = 0, (x y) is ON the line. 
  // If F(x y) > 0, (x y) is "above" the line. 
  // If F(x y) < 0, (x y) is "below" the line.

  double xyl = (p2y - p1y) * xLower + (p1x - p2x) * yLower + (p2x * p1y - p1x * p2y);
  double xlyu = (p2y - p1y) * xLower + (p1x - p2x) * yUpper + (p2x * p1y - p1x * p2y);
  double xyu = (p2y - p1y) * xUpper + (p1x - p2x) * yUpper + (p2x * p1y - p1x * p2y);
  double xuyl = (p2y - p1y) * xUpper + (p1x - p2x) * yLower + (p2x * p1y - p1x * p2y);
  if (xyl > 0 && xlyu > 0 && xyu > 0 && xuyl > 0) {
    return false;
  }
  if (xyl < 0 && xlyu < 0 && xyu < 0 && xuyl < 0) {
    return false;
  }

  return true;
}

unsigned int detectNodeCollisions(QuadtreeNode* quadtreeNode, IntersectionEventList* intersectionEventList) {
  return 0;
}






