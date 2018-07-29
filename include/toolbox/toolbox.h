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
    void add(std::string name, boost::shared_ptr<global_planner::Plugin> p);

  template <typename T>
  boost::shared_ptr<T> get(std::string s) {
    if(s == "GVD"){
      boost::shared_ptr<global_planner::Plugin> p;
      assert(p != NULL);
      boost::shared_ptr<T> p1 = boost::dynamic_pointer_cast<T>(p);
      assert(p1 != NULL);
      return p1;
    }
  }
  

  private:
    std::unordered_map<std::string, boost::shared_ptr<global_planner::Plugin> > map_;
};
}



#endif