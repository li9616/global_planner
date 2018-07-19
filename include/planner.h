#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <costmap_2d/costmap_2d.h>

#include "constants.h"
#include "helper.h"
#include "collisiondetection.h"
#include "dynamicvoronoi.h"
#include "algorithm/algorithm.h"
#include "pose2d.h"
#include "lookup.h"

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

  Planner(costmap_2d::Costmap2D* costmap, std::vector<geometry_msgs::Point> footprint_spec, unsigned int cell_divider);

  /*!
     \brief Sets the map e.g. through a callback from a subscriber listening to map updates.
     \param map the map or occupancy grid
  */
  void setMapfromParam(costmap_2d::Costmap2D* costmap);

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

  void plan(costmap_2d::Costmap2D* costmap, 
            const geometry_msgs::PoseStamped start, 
            const geometry_msgs::PoseStamped goal, 
            std::vector<geometry_msgs::PoseStamped>& result_path);


  void tracePath(const Pose2D* node, int i, std::vector<Pose2D>& path);

  nav_msgs::Path path_;

 private:

  /// The collission detection for testing specific configurations
  HybridAStar::CollisionDetection configurationSpace;
  /// The voronoi diagram
  HybridAStar::DynamicVoronoi voronoiDiagram;
  /// A pointer to the grid the planner runs on
  nav_msgs::OccupancyGrid::Ptr grid;
  /// The start pose set through RViz
  geometry_msgs::PoseStamped start;
  /// The goal pose set through RViz
  geometry_msgs::PoseStamped goal;
  /// Flags for allowing the planner to plan
  bool validStart = false;
  /// Flags for allowing the planner to plan
  bool validGoal = false;
  /// A lookup table for configurations of the vehicle and their spatial occupancy enumeration
  Constants::config collisionLookup[Constants::headings * Constants::positions];
  /// A lookup of analytical solutions (Dubin's paths)
  // float* dubinsLookup = new float [Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];

  bool** binMap_;//YT 用于存储二进制格式的地图
  unsigned char** charMap_;//YT 用于存储八位占用率地图
  

  Algorithm::Algorithm* yt_alg_;
  costmap_2d::Costmap2D* costmap_;
  std::vector<geometry_msgs::Point> footprint_spec_;
  unsigned int cell_divider_;
  
};
}
#endif // PLANNER_H
