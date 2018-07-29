#ifndef _ASTAR_
#define _ASTAR_

#include "algorithm/algorithm.h"
#include "base_type/node2d.h"
#include "toolbox/collisiondetection/collisiondetection.h"
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/PoseArray.h>


namespace yt{
class AStar : public Algorithm
{
public:
AStar(std::string frame_id, double origin_position_x, double origin_position_y, double gridmap_resolution):
  Algorithm(frame_id, origin_position_x, origin_position_y, gridmap_resolution)
{
    ros::NodeHandle private_nh("~");

    mid_result_pub_ = private_nh.advertise<geometry_msgs::PoseArray>("astar_planner/mid_result", 1);

}

virtual bool plan(global_planner::Pose2D& start, 
                    const global_planner::Pose2D& goal, 
                    global_planner::Pose2D* nodes3D, 
                    global_planner::Node2D* nodes2D, 
                    int width, 
                    int height, 
                    boost::shared_ptr<CollisionDetection> configurationSpace, 
                    boost::shared_ptr<global_planner::DynamicVoronoi> voronoiDiagram, 
                    std::vector<global_planner::Pose2D>& plan);
  // virtual bool updateH(global_planner::Pose2D& start, const global_planner::Pose2D& goal, global_planner::Node2D* nodes2D, int width, int height, CollisionDetection& configurationSpace);


};



}//namespace

#endif