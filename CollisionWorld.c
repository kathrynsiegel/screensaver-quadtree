/** 
 * CollisionWorld.c -- detect and handle line segment intersections
 * Copyright (c) 2012 the Massachusetts Institute of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 **/

#include "CollisionWorld.h"

#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>

#include "IntersectionDetection.h"
#include "IntersectionEventList.h"
#include "Line.h"
#include "Quadtree.h"

#include <cilk/cilk.h>
#include <cilk/reducer.h>
#include <cilk/reducer_opadd.h>

CollisionWorld* CollisionWorld_new(const unsigned int capacity) {
  assert(capacity > 0);

  CollisionWorld* collisionWorld = malloc(sizeof(CollisionWorld));
  if (collisionWorld == NULL) {
    return NULL;
  }

  collisionWorld->numLineWallCollisions = 0;
  collisionWorld->numLineLineCollisions = 0;
  collisionWorld->timeStep = 0.5;
  collisionWorld->lines = malloc(capacity * sizeof(Line*));
  collisionWorld->numOfLines = 0;
  collisionWorld->quadtree = Quadtree_new(collisionWorld, Vec_make(BOX_XMIN,BOX_YMIN), Vec_make(BOX_XMAX,BOX_YMAX), 0, NULL);
  
  return collisionWorld;
}

void CollisionWorld_delete(CollisionWorld* collisionWorld) {
  for (int i = 0; i < collisionWorld->numOfLines; i++) {
    free(collisionWorld->lines[i]);
  }
  free(collisionWorld->lines);
  Quadtree_delete(collisionWorld->quadtree);
  free(collisionWorld);
}

unsigned int CollisionWorld_getNumOfLines(CollisionWorld* collisionWorld) {
  return collisionWorld->numOfLines;
}

void CollisionWorld_addLine(CollisionWorld* collisionWorld, Line *line) {
  // precalculate the length of the line
  line->length = Vec_length(Vec_subtract(line->p1, line->p2));
  
  // precalculate the parallelogram created by initial velocity
  updateParallelogram(line, collisionWorld->timeStep);
  
  collisionWorld->lines[collisionWorld->numOfLines] = line;
  collisionWorld->numOfLines++;
  // recreate the quadtree (this setup is called before the timed portion)
  Quadtree_delete(collisionWorld->quadtree);
  collisionWorld->quadtree = Quadtree_new(collisionWorld, Vec_make(BOX_XMIN,BOX_YMIN), Vec_make(BOX_XMAX,BOX_YMAX), collisionWorld->numOfLines, NULL);
}

Line* CollisionWorld_getLine(CollisionWorld* collisionWorld,
                             const unsigned int index) {
  if (index >= collisionWorld->numOfLines) {
    return NULL;
  }
  return collisionWorld->lines[index];
}

void CollisionWorld_updateLines(CollisionWorld* collisionWorld) {
  CollisionWorld_detectIntersection(collisionWorld);
  CollisionWorld_updatePosition(collisionWorld);
  CollisionWorld_lineWallCollision(collisionWorld);
}

void CollisionWorld_updatePosition(CollisionWorld* collisionWorld) {
  double t = collisionWorld->timeStep;
  cilk_for (int i = 0; i < collisionWorld->numOfLines; i++) {
    Line *line = collisionWorld->lines[i];
    line->p1 = Vec_add(line->p1, Vec_multiply(line->velocity, t));
    line->p2 = Vec_add(line->p2, Vec_multiply(line->velocity, t));
  }
}

void CollisionWorld_lineWallCollision(CollisionWorld* collisionWorld) {
  CILK_C_REDUCER_OPADD(numCollisionsReducer, int, 0);
  CILK_C_REGISTER_REDUCER(numCollisionsReducer);
  cilk_for (int i = 0; i < collisionWorld->numOfLines; i++) {
    Line *line = collisionWorld->lines[i];
    bool collide = false;

    // Right side
    if ((line->p1.x > BOX_XMAX || line->p2.x > BOX_XMAX)
        && (line->velocity.x > 0)) {
      line->velocity.x = -line->velocity.x;
      collide = true;
    }
    // Left side
    if ((line->p1.x < BOX_XMIN || line->p2.x < BOX_XMIN)
        && (line->velocity.x < 0)) {
      line->velocity.x = -line->velocity.x;
      collide = true;
    }
    // Top side
    if ((line->p1.y > BOX_YMAX || line->p2.y > BOX_YMAX)
        && (line->velocity.y > 0)) {
      line->velocity.y = -line->velocity.y;
      collide = true;
    }
    // Bottom side
    if ((line->p1.y < BOX_YMIN || line->p2.y < BOX_YMIN)
        && (line->velocity.y < 0)) {
      line->velocity.y = -line->velocity.y;
      collide = true;
    }
    // Update total number of collisions.
    if (collide == true) {
      REDUCER_VIEW(numCollisionsReducer)++;
    }
    
    // precalculate the parallelogram created by final velocity
    updateParallelogram(line, collisionWorld->timeStep);
  }
  collisionWorld->numLineWallCollisions += REDUCER_VIEW(numCollisionsReducer);
  CILK_C_UNREGISTER_REDUCER(numCollisionsReducer);
}

void CollisionWorld_detectIntersection(CollisionWorld* collisionWorld) {
  IntersectionEventListReducer intersectionEventListReducer = CILK_C_INIT_REDUCER(/* type */ IntersectionEventList,
  intersection_event_list_reduce, intersection_event_list_identity, intersection_event_list_destroy,
  /* initial value */ (IntersectionEventList) { .head = NULL, .tail = NULL });

  CILK_C_REGISTER_REDUCER(intersectionEventListReducer);
  
  Quadtree_update(collisionWorld->quadtree);
  
  CILK_C_REDUCER_OPADD(numCollisionsReducer, int, 0);
  CILK_C_REGISTER_REDUCER(numCollisionsReducer);
  
  detectCollisionsReducer(collisionWorld->quadtree, &intersectionEventListReducer, &numCollisionsReducer);
  
  int numCollisions = REDUCER_VIEW(numCollisionsReducer);
  
  CILK_C_UNREGISTER_REDUCER(numCollisionsReducer);
  
  
  IntersectionEventList intersectionEventList = REDUCER_VIEW(intersectionEventListReducer);
  
  // Sort the intersection event list.
  IntersectionEventNode* startNode = intersectionEventList.head;
  while (startNode != NULL) {
    IntersectionEventNode* minNode = startNode;
    IntersectionEventNode* curNode = startNode->next;
    IntersectionEventNode* prevNode = startNode;
    IntersectionEventNode* delNode;
    while (curNode != NULL) {
      int comp = IntersectionEventNode_compareData(curNode, minNode);
      if (comp == 0) {
        delNode = curNode;
        curNode = curNode->next;
        prevNode->next = curNode;
        free(delNode);
        numCollisions--;
      } else {
        if (comp < 0) {
          minNode = curNode;
        }
        prevNode = curNode;
        curNode = curNode->next;
      }
    }
    if (minNode != startNode) {
      IntersectionEventNode_swapData(minNode, startNode);
    }
    CollisionWorld_collisionSolver(collisionWorld, startNode->l1, startNode->l2,
                                   startNode->intersectionType);
    startNode = startNode->next;
  }

  collisionWorld->numLineLineCollisions += numCollisions;

  CILK_C_UNREGISTER_REDUCER(intersectionEventListReducer);
}

unsigned int CollisionWorld_getNumLineWallCollisions(
    CollisionWorld* collisionWorld) {
  return collisionWorld->numLineWallCollisions;
}

unsigned int CollisionWorld_getNumLineLineCollisions(
    CollisionWorld* collisionWorld) {
  return collisionWorld->numLineLineCollisions;
}

void CollisionWorld_collisionSolver(CollisionWorld* collisionWorld,
                                    Line *l1, Line *l2,
                                    IntersectionType intersectionType) {
  assert(compareLines(l1, l2) < 0);
  assert(intersectionType == L1_WITH_L2
         || intersectionType == L2_WITH_L1
         || intersectionType == ALREADY_INTERSECTED);

  // Despite our efforts to determine whether lines will intersect ahead
  // of time (and to modify their velocities appropriately), our
  // simplified model can sometimes cause lines to intersect.  In such a
  // case, we compute velocities so that the two lines can get unstuck in
  // the fastest possible way, while still conserving momentum and kinetic
  // energy.
  if (intersectionType == ALREADY_INTERSECTED) {
    Vec p = getIntersectionPoint(l1->p1, l1->p2, l2->p1, l2->p2);
    double l1_vel_len = Vec_length(l1->velocity);
    double l2_vel_len = Vec_length(l2->velocity);

    if (Vec_length(Vec_subtract(l1->p1, p))
        < Vec_length(Vec_subtract(l1->p2, p))) {
      l1->velocity = Vec_multiply(Vec_normalize(Vec_subtract(l1->p2, p)),
                                  l1_vel_len);
    } else {
      l1->velocity = Vec_multiply(Vec_normalize(Vec_subtract(l1->p1, p)),
                                  l1_vel_len);
    }
    if (Vec_length(Vec_subtract(l2->p1, p))
        < Vec_length(Vec_subtract(l2->p2, p))) {
      l2->velocity = Vec_multiply(Vec_normalize(Vec_subtract(l2->p2, p)),
                                  l2_vel_len);
    } else {
      l2->velocity = Vec_multiply(Vec_normalize(Vec_subtract(l2->p1, p)),
                                  l2_vel_len);
    }
    return;
  }

  // Compute the collision face/normal vectors.
  Vec face;
  Vec normal;
  if (intersectionType == L1_WITH_L2) {
    Vec v = Vec_makeFromLine(*l2);
    face = Vec_normalize(v);
  } else {
    Vec v = Vec_makeFromLine(*l1);
    face = Vec_normalize(v);
  }
  normal = Vec_orthogonal(face);

  // Obtain each line's velocity components with respect to the collision
  // face/normal vectors.
  double v1Face = Vec_dotProduct(l1->velocity, face);
  double v2Face = Vec_dotProduct(l2->velocity, face);
  double v1Normal = Vec_dotProduct(l1->velocity, normal);
  double v2Normal = Vec_dotProduct(l2->velocity, normal);

  // Compute the mass of each line (we simply use its length).
  double m1 = l1->length;
  double m2 = l2->length;

  // Perform the collision calculation (computes the new velocities along
  // the direction normal to the collision face such that momentum and
  // kinetic energy are conserved).
  double newV1Normal = ((m1 - m2) / (m1 + m2)) * v1Normal
      + (2 * m2 / (m1 + m2)) * v2Normal;
  double newV2Normal = (2 * m1 / (m1 + m2)) * v1Normal
      + ((m2 - m1) / (m2 + m1)) * v2Normal;

  // Combine the resulting velocities.
  l1->velocity = Vec_add(Vec_multiply(normal, newV1Normal),
                         Vec_multiply(face, v1Face));
  l2->velocity = Vec_add(Vec_multiply(normal, newV2Normal),
                         Vec_multiply(face, v2Face));

  return;
}
