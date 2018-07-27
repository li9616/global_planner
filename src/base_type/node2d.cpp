#include "base_type/node2d.h"

using namespace global_planner;

// // possible directions
// int dir = 8;
// possible movements
int node2d_dx[] = { -1, -1, 0, 1, 1, 1, 0, -1 };
int node2d_dy[] = { 0, 1, 1, 1, 0, -1, -1, -1 };


//###################################################
//                                   CREATE SUCCESSOR
//###################################################
Node2D* Node2D::createSuccessor(const int i) {
  int xSucc = x + node2d_dx[i];
  int ySucc = y + node2d_dy[i];
  return new Node2D(xSucc, ySucc, g, 0, this);
}

//###################################################
//                                 2D NODE COMPARISON
//###################################################
bool Node2D::operator == (const Node2D& rhs) const {
  return x == rhs.x && y == rhs.y;
}

