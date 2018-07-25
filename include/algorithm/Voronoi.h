#ifndef _VORONOI_
#define _VORONOI_

#include "algorithm/algorithm.h"
#include "collisiondetection/collisiondetection.h"
namespace yt {

class Voronoi : public Algorithm
{
public:

Voronoi(bool using_voronoi):Algorithm()
{
    using_voronoi_ = using_voronoi;
}

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

private:

    std::vector<std::pair<float, float> > path1_;
    std::vector<std::pair<float, float> > path2_;
    std::vector<std::pair<float, float> > path3_;

    bool using_voronoi_;

};

}//namespace
#endif