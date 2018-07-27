#ifndef _ASTAR_
#define _ASTAR_

#include "algorithm/algorithm.h"
#include "base_type/node2d.h"
#include "toolbox/collisiondetection/collisiondetection.h"
namespace yt{
class AStar : public Algorithm
{
public:
AStar(){}

virtual bool plan(global_planner::Pose2D& start, 
                    const global_planner::Pose2D& goal, 
                    global_planner::Pose2D* nodes3D, 
                    global_planner::Node2D* nodes2D, 
                    int width, 
                    int height, 
                    CollisionDetection* configurationSpace, 
                    boost::shared_ptr<global_planner::DynamicVoronoi> voronoiDiagram, 
                    std::vector<global_planner::Pose2D>& plan);
  // virtual bool updateH(global_planner::Pose2D& start, const global_planner::Pose2D& goal, global_planner::Node2D* nodes2D, int width, int height, CollisionDetection& configurationSpace);


};



}//namespace

#endif