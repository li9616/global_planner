#include "base_type/pose2d.h"
#include <cmath>

using namespace global_planner;

// CONSTANT VALUES
// possible directions
// int Pose2D::dir = 3;

// R = 6, 6.75 DEG
float pose2d_dy[] = { 0,   1,  -1, 0};
float pose2d_dx[] = { 1,   0,   0, -1};
float pose2d_dt[] = { 0,   M_PI/2,   -M_PI/2, M_PI};


//###################################################
//                                   CREATE SUCCESSOR
//###################################################
Pose2D* Pose2D::createSuccessor(const int i) {
  float xSucc;
  float ySucc;
  float tSucc;

  // calculate successor positions forward
  // if (i < 3) {
    xSucc = x + pose2d_dx[i] * cos(t) - pose2d_dy[i] * sin(t);
    ySucc = y + pose2d_dx[i] * sin(t) + pose2d_dy[i] * cos(t);
    tSucc = Helper::normalizeHeadingRad(t + pose2d_dt[i]);


  return new Pose2D(xSucc, ySucc, tSucc, g, 0, this, i);
}


//###################################################
//                                      MOVEMENT COST
//###################################################
void Pose2D::updateG() {
  // forward driving
  if (prim < 3) {
    // penalize turning
    if (pred->prim != prim) {
      // penalize change of direction
      if (pred->prim > 2) {
        g += pose2d_dx[0] * Constants::penaltyTurning * Constants::penaltyCOD;
      } else {
        g += pose2d_dx[0] * Constants::penaltyTurning;
      }
    } else {
      g += pose2d_dx[0];
    }
  }
  // reverse driving
  else {
    // penalize turning and reversing
    if (pred->prim != prim) {
      // penalize change of direction
      if (pred->prim < 3) {
        g += pose2d_dx[0] * Constants::penaltyTurning * Constants::penaltyReversing * Constants::penaltyCOD;
      } else {
        g += pose2d_dx[0] * Constants::penaltyTurning * Constants::penaltyReversing;
      }
    } else {
      g += pose2d_dx[0] * Constants::penaltyReversing;
    }
  }
}


//###################################################
//                                 3D NODE COMPARISON
//###################################################
bool Pose2D::operator == (const Pose2D& rhs) const {
  return (int)x == (int)rhs.x &&
         (int)y == (int)rhs.y &&
         (std::abs(t - rhs.t) <= Constants::deltaHeadingRad ||
          std::abs(t - rhs.t) >= Constants::deltaHeadingNegRad);
}
