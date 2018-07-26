#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "base_type/pose2d.h"
#include "base_type/node2d.h"
#include "toolbox/collisiondetection/collisiondetection.h"
#include "base_type/primitive.h"
#include "toolbox/voronoi/dynamicvoronoi.h"
#include <ros/ros.h>

namespace yt {

/*!
 * \brief A class that encompasses the functions central to the search.
 */
class Algorithm {
 public:
  /// The deault constructor
  Algorithm() {}

  /*!
     \brief The heart of the planner, the main algorithm starting the search for a collision free and drivable path.

     \param start the start pose
     \param goal the goal pose
     \param nodes3D the array of 3D nodes representing the configuration space C in R^3
     \param nodes2D the array of 2D nodes representing the configuration space C in R^2
     \param width the width of the grid in number of cells
     \param height the height of the grid in number of cells
     \param configurationSpace the lookup of configurations and their spatial occupancy enumeration
     \return the pointer to the node satisfying the goal condition
  */
  virtual bool plan(global_planner::Pose2D& start, 
                    const global_planner::Pose2D& goal, 
                    global_planner::Pose2D* nodes3D, 
                    global_planner::Node2D* nodes2D, 
                    int width, 
                    int height, 
                    CollisionDetection* configurationSpace, 
                    global_planner::DynamicVoronoi* voronoiDiagram,
                    std::vector<global_planner::Pose2D>& plan) = 0;
  //  virtual bool updateH(global_planner::Pose2D& start, const global_planner::Pose2D& goal, global_planner::Node2D* nodes2D, int width, int height, CollisionDetection& configurationSpace);


  std::vector<global_planner::Pose2D> mid_result;//YT 存放搜索的中间结果，格式是Pose2D，在planner外再转成PoseArray去显示

protected:
  std::vector<primitive> motion_primitive_;

  ros::NodeHandle private_nh;

};
}//namespace
#endif // ALGORITHM_H
