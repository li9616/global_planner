#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <costmap_2d/costmap_2d.h>

#include "constants.h"
#include "helper.h"
#include "toolbox/collisiondetection/collisiondetection.h"
#include "toolbox/voronoi/dynamicvoronoi.h"
#include "algorithm/algorithm.h"
#include "base_type/pose2d.h"
#include "boost/shared_ptr.hpp"
#include <string>


namespace global_planner {
/*!
   \brief A class that creates the interface for the hybrid A* algorithm.

    It inherits from `ros::nav_core::BaseGlobalPlanner` so that it can easily be used with the ROS navigation stack
   \todo make it actually inherit from nav_core::BaseGlobalPlanner
*/
class Planner {
 public:
  // /// The default constructor
  // Planner();

  Planner(costmap_2d::Costmap2D* costmap, 
          std::vector<geometry_msgs::Point> footprint_spec, 
          unsigned int cell_divider, 
          bool using_voronoi, 
          bool lazy_replanning, 
          std::string frame_id);

  ~Planner();

  /*!
     \brief Sets the map e.g. through a callback from a subscriber listening to map updates.
     \param map the map or occupancy grid
  */
  void setupCollisionDetection(costmap_2d::Costmap2D* costmap);

  /*!
     \brief setStart
     \param start the start pose
  */
  void setStartfromParam(const geometry_msgs::PoseStamped start);

  /*!
     \brief setGoal
     \param goal the goal pose
  */
  void setGoalfromParam(const geometry_msgs::PoseStamped end);

  /*!
     \brief The central function entry point making the necessary preparations to start the planning.
  */
  void plan(std::vector<geometry_msgs::PoseStamped>& result_path);

  void plan(const geometry_msgs::PoseStamped start, 
            const geometry_msgs::PoseStamped goal, 
            std::vector<geometry_msgs::PoseStamped>& result_path);


  void tracePath(const Pose2D* node, int i, std::vector<Pose2D>& path);
  
  void visualizeBinMap(const char* filename);
  
  boost::shared_ptr<DynamicVoronoi> getVoronoi();

  nav_msgs::Path path_;

  // geometry_msgs::PoseArray mid_result;
  geometry_msgs::PoseArray vor_path_result;
 private:

  geometry_msgs::PoseStamped start;
  geometry_msgs::PoseStamped goal;

  //YT costmap相关
  costmap_2d::Costmap2D* costmap_;
  bool** binMap_;//YT 用于存储二进制格式的地图, true = free, false = occupied
  bool** binMap_not_;//true = occupied, false = free;
  unsigned char** charMap_;//YT 用于存储八位占用率地图


  double origin_position_x_;
  double origin_position_y_;
  double origin_position_z_;
  double origin_orientation_x_;
  double origin_orientation_y_;
  double origin_orientation_z_;
  double origin_orientation_w_;

  double costmap_width_x_;
  double costmap_height_y_;
  double costmap_resolution_;//YT costmap一个网格的边长大小，单位是m


//YT gridmap相关
  unsigned int cell_divider_;
  unsigned int gridmap_width_x_;
  unsigned int gridmap_height_y_;
  double gridmap_resolution_;//YT todo: 删除gridmap的实例


  yt::Algorithm *yt_alg_, *yt_alg_1;
  
  //YT 碰撞检测plugin相关
  boost::shared_ptr<CollisionDetection> configurationSpace_;
  // CollisionDetection* configurationSpace;

  std::vector<geometry_msgs::Point> footprint_spec_;

  //YT voronoi图plugin相关
  // DynamicVoronoi* voronoiDiagram;
  boost::shared_ptr<DynamicVoronoi> voronoiDiagram;



  bool using_voronoi_;
  std::string frame_id_;
};
}
#endif // PLANNER_H
