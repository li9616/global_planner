#include "algorithm/AStar.h"

#include "base_type/node2d.h"
#include "base_type/pose2d.h"

#include "algorithm/algorithm.h"
#include <Eigen/Dense>
#include <boost/heap/binomial_heap.hpp>
#include <boost/shared_ptr.hpp>


using namespace yt;

void tracePath(const global_planner::Node2D* node, int i, std::vector<global_planner::Pose2D>& path);

bool isOnGrid(const global_planner::Node2D node, const int width, const int height);
//###################################################
//                                    NODE COMPARISON
//###################################################
/*!
   \brief A structure to sort nodes in a heap structure
*/
struct CompareNodes{
/// Sorting 2D nodes by increasing C value - the total estimated cost
bool operator()(global_planner::Node2D* lhs, global_planner::Node2D* rhs) const{
  return lhs->getC() > rhs->getC();
}
};

bool isOnGrid(const global_planner::Node2D node, const int width, const int height){
  return  node.getX() >= 0 && node.getX() < width && node.getY() >= 0 && node.getY() < height;
}


//###################################################
//                                        2D A*
//###################################################
bool yt::AStar::plan(global_planner::Pose2D& start_temp, 
                    const global_planner::Pose2D& goal_temp, 
                    global_planner::Pose2D* nodes3D_temp, 
                    global_planner::Node2D* nodes2D, 
                    int width, 
                    int height, 
                    boost::shared_ptr<CollisionDetection> configurationSpace, 
                    boost::shared_ptr<global_planner::DynamicVoronoi> voronoiDiagram, 
                    std::vector<global_planner::Pose2D>& plan){

  ////////////////////////////////////
  //YT 临时做一个接口，去掉机器人朝向分量，从pose转成position来做普通A*

  global_planner::Node2D start;
  global_planner::Node2D goal;
  // global_planner::Pose2D* nodes3D;//YT 传统A*不需要用到
  // global_planner::Nodes2D* nodes2D;//YT 直接用就可以了



  start.setX(start_temp.getX());
  start.setY(start_temp.getY());
  goal.setX(goal_temp.getX());
  goal.setY(goal_temp.getY());

  //////////////////////////////////////
  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;
  int dir = 8;
  // reset the open and closed list
  for (int i = 0; i < width * height; ++i) {
    nodes2D[i].reset();
  }

  // VISUALIZATION DELAY
  ros::Duration d(0.001);

  boost::heap::binomial_heap<global_planner::Node2D*, boost::heap::compare<CompareNodes>> O;

  // update h value
  start.updateH(goal);

  // mark start as open
  start.open();
  // push on priority queue
  O.push(&start);
  iPred = start.setIdx(width);

  nodes2D[iPred] = start;

  // NODE POINTER
  global_planner::Node2D* nPred;
  global_planner::Node2D* nSucc;

  mid_result_.clear();

  // continue until O empty
  while (!O.empty()) {
    // pop node with lowest cost from priority queue
    nPred = O.top();
    // set index
    iPred = nPred->setIdx(width);

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes2D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes2D[iPred].isOpen()) {
      // add node to closed list
      nodes2D[iPred].close();
      nodes2D[iPred].discover();//YT 把成员变量d变成true

      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST
      if (*nPred == goal) {
        //YT 找到了目标路径，要通过plan参数传回去
        // std::cout << "YT: A* has found the goal" << std::endl;

        ///////////////////////////////////////////////////
        tracePath(nPred, 0, plan);//YT 从链表返回出plan，但是plan里面的东西是不带起终点，而且顺序是反着的
        publishMidResult(mid_result_);
        return !plan.empty();
        //////////////////////////////////////////////////
      }
      // ____________________
      // CONTINUE WITH SEARCH
      else {
        // _______________________________
        // CREATE POSSIBLE SUCCESSOR NODES
        for (int i = 0; i < dir; i++) {
          // create possible successor
          nSucc = nPred->createSuccessor(i);

          nPred->after_rot_ = global_planner::Helper::normalizeHeadingRad(atan2((nSucc->getY() - nPred->getY()), (nSucc->getX() - nPred->getX())));
          nSucc->before_rot_ =global_planner::Helper::normalizeHeadingRad(atan2((nSucc->getY() - nPred->getY()), (nSucc->getX() - nPred->getX())));
           

          //YT save mid_result_//////////////////
          global_planner::Pose2D temp;
          temp.setX(nSucc->getX());
          temp.setY(nSucc->getY());
          temp.setT(global_planner::Helper::normalizeHeadingRad(atan2((nSucc->getY() - nPred->getY()), (nSucc->getX() - nPred->getX()))) );//YT orientation before rotation
          mid_result_.push_back(temp);
          ////////////////////////////////

          // set index of the successor
          iSucc = nSucc->setIdx(width);


          // YT: ensure the rotation at nPred is valid
          // YT: ensure the movement between nPred and nSucc is valid
          // ensure successor is on grid ROW MAJOR
          // ensure successor is not on closed list
          if (configurationSpace->isTraversableForRotation(nPred) &&
              configurationSpace->isTraversableForMovement(nPred, nSucc) &&
              isOnGrid(*nSucc, width, height) &&
              !nodes2D[iSucc].isClosed()) {
            // std::cout << "YT: no collision" << std::endl;
            // calculate new G value
            nSucc->updateG();
            newG = nSucc->getG();

            // if successor not on open list or g value lower than before put it on open list
            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG()) {
              // calculate the H value
              nSucc->updateH(goal);
              // put successor on open list
              nSucc->open();
              nodes2D[iSucc] = *nSucc;
              O.push(&nodes2D[iSucc]);
              delete nSucc;

            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }
  // return large number to guide search away
  return false;
}

void tracePath(const global_planner::Node2D* node, int i, std::vector<global_planner::Pose2D>& path)
{//YT still in gridmap frame
    if (node == nullptr) {
      // std::cout << "YT: maybe no path to trace" << std::endl;
    return;
  }
  i++;
  global_planner::Pose2D pose_temp;
  pose_temp.setX(node->getX());
  pose_temp.setY(node->getY());
  path.push_back(pose_temp);
  tracePath(node->getPred(), i, path);
}
