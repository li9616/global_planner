#ifndef _TOOLBOX_
#define _TOOLBOX_

#include "toolbox/voronoi/dynamicvoronoi.h"
#include <unordered_map>
#include <string>
#include <boost/shared_ptr.hpp>
#include <memory>

namespace global_planner{


class Toolbox {
  public:
//   static Toolbox* getInstance();

    void start();

    template <typename T>    
    void add(std::string name, costmap_2d::Costmap2D* costmap, int cell_divider, std::string frame_id){
      std::pair<std::unordered_map<std::string, boost::shared_ptr<global_planner::Plugin> > ::iterator, bool> res = 
          map_.insert(std::make_pair<std::string, boost::shared_ptr<global_planner::Plugin> >
                                                  (name, new T(costmap, cell_divider, frame_id)));
      if(!res.second)
      {
        std::cout << "YT: fail to insert new plugin into toolbox" << std::endl;
      }
    }

    template <typename T>
    boost::shared_ptr<T> get(std::string name) {

      std::unordered_map<std::string, boost::shared_ptr<global_planner::Plugin> >::iterator iter;
      iter = map_.find(name);
      assert(iter != map_.end());
      assert(iter->second != NULL);

      boost::shared_ptr<T> p1 = boost::dynamic_pointer_cast<T>(iter->second);
      assert(p1 != NULL);
      return p1;
    }
  

  private:
    std::unordered_map<std::string, boost::shared_ptr<global_planner::Plugin> > map_;
};
}



#endif