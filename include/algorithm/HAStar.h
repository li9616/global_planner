#ifndef _HASTAR_
#define _HASTAR_

#include "algorithm/algorithm.h"
#include "toolbox/collisiondetection/collisiondetection.h"
#include <string>

namespace yt{
class HAStar : public Algorithm
{
public:

HAStar(std::string frame_id, double origin_position_x, double origin_position_y, double gridmap_resolution):
  Algorithm(frame_id, origin_position_x, origin_position_y, gridmap_resolution)
{}

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