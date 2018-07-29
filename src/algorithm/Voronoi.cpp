#include "algorithm/Voronoi.h"
#include <boost/shared_ptr.hpp>


using namespace yt;

bool yt::Voronoi::plan(global_planner::Pose2D& start_temp, 
                    const global_planner::Pose2D& goal_temp, 
                    global_planner::Pose2D* nodes3D_temp, 
                    global_planner::Node2D* nodes2D, 
                    int width, 
                    int height, 
                    boost::shared_ptr<CollisionDetection> configurationSpace, 
                    boost::shared_ptr<global_planner::DynamicVoronoi> voronoiDiagram, 
                    std::vector<global_planner::Pose2D>& plan){



if(!using_voronoi_){
    ROS_ERROR("YT: Voronoi_planner cannot run without voronoi_graph plugin");
    return false;
}

    return voronoiDiagram->getPathInVoronoi(
        (int)start_temp.getX(), (int)start_temp.getY(), (int)goal_temp.getX(), (int)goal_temp.getY(), plan);

}
