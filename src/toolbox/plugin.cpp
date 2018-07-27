#include "toolbox/plugin.h"
#include "costmap_2d/costmap_2d_ros.h"
#include <string>

using namespace global_planner;

Plugin::Plugin(costmap_2d::Costmap2D* costmap){
    costmap_ = costmap;
}

Plugin::Plugin(costmap_2d::Costmap2D* costmap, int cell_divider, std::string frame_id){
    costmap_ = costmap;
    cell_divider_ = cell_divider;
    frame_id_ = frame_id;
}
