#ifndef _HASTAR_
#define _HASTAR_

#include "algorithm/algorithm.h"
#include "collisiondetection/collisiondetection.h"
namespace yt{
class HAStar : public Algorithm
{
public:

HAStar(){}

virtual bool plan(global_planner::Pose2D& start, 
                    const global_planner::Pose2D& goal, 
                    global_planner::Pose2D* nodes3D, 
                    global_planner::Node2D* nodes2D, 
                    int width, 
                    int height, 
                    CollisionDetection* configurationSpace, 
                    global_planner::DynamicVoronoi* voronoiDiagram, 
                    std::vector<global_planner::Pose2D>& plan);
  // virtual bool updateH(global_planner::Pose2D& start, const global_planner::Pose2D& goal, global_planner::Node2D* nodes2D, int width, int height, CollisionDetection& configurationSpace);

};

}//namespace
#endif