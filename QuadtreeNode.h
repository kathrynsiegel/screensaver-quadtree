/**
 * QuadtreeNode.h -- Quadtree node data structure to improve collision detections
 *
 **/

#include "Line.h"
#include "Vec.h"
#include "IntersectionEventList.h"
#include "CollisionWorld.h"

#define MAX_LINES_PER_NODE 10

#ifndef QUADTREENODE_H_
#define QUADTREENODE_H_

typedef struct QuadtreeNode QuadtreeNode;

struct QuadtreeNode {
  // The CollisionWorld the Quadtree exists in
  CollisionWorld* collisionWorld;

  // The upper left corner of the Quadtree node
  Vec* upperLeft;
  
  // The lower right corner of the Quadtree node
  Vec* lowerRight;

  // Array containing all of the lines that are part of this level of the Quadtree
  Line** lines;
  unsigned int numOfLines;

  // Array containing four quadrants of this Quadtree
  QuadtreeNode** quadrants;
  
  // True if the Quadtree contains less than MAX_LINES_PER_NODE lines
  bool isLeaf;
};

QuadtreeNode* QuadtreeNode_make(CollisionWorld* collisionWorld, Vec* upperLeft, Vec* lowerRight, Line** lines, unsigned int numLines); 

void QuadtreeNode_delete();

// Populates the quad tree node with lines and then recursively creates new nodes
void populateQuadtreeNode(QuadtreeNode* quadtreeNode);

unsigned int detectNodeCollisions(QuadtreeNode* quadtreeNode, IntersectionEventList* intersectionEventList);

bool testInBox(double xLower, double xUpper, double yLower, double yUpper, Line* line);

bool testLinesStraddleBox(double xLower, double xUpper, double yLower, double yUpper, Line* line, Line* movedLine);

bool testLineInBox(double xLower, double xUpper, double yLower, double yUpper, Line* line);

#endif
