#ifndef _PLUGIN_
#define _PLUGIN_

#include <boost/shared_ptr.hpp>
#include <string>
#include <costmap_2d/costmap_2d_ros.h>

namespace global_planner {
    
class Plugin
{
public:
  typedef boost::shared_ptr<Plugin> Ptr;

  Plugin(costmap_2d::Costmap2D* costmap, int cell_divider, std::string frame_id );
  Plugin(costmap_2d::Costmap2D* costmap);

protected:
  std::string frame_id_;
  costmap_2d::Costmap2D* costmap_;
  int cell_divider_;
  

};


}

#endif