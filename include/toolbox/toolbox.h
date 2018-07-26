#ifndef _TOOLBOX_
#define _TOOLBOX_

#include "toolbox/voronoi/dynamicvoronoi.h"
#include <unordered_map>
#include <string>

namespace global_planner{


class Toolbox {
  public:
//   static Toolbox* getInstance();

//     void add(Plugin::Ptr p);

//     template <typename T>
//     boost::shared_ptr<T> get(std::string s){
//         Plugin::Ptr p = map[s];
//         assert(p != NULL);
//         boost::shared_ptr<T> p1 = boost::dynamic_cast<T>(p);
//         assert(p1 != NULL);
//         return p1;

//     }
    void start();

  private:
    std::unordered_map<std::string, Plugin::Ptr> map_;
};
}



#endif