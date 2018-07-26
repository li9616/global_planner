cmake_minimum_required(VERSION 2.8.3)
project(global_planner)

## C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED
  COMPONENTS
    angles
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf
    base_local_planner
    visualization_msgs
)

generate_dynamic_reconfigure_options(
  cfg/GlobalPlanner.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf
    pcl
)


## OPEN MOTION PLANNING LIBRARY
# find_package(OMPL REQUIRED)

# if(NOT OMPL_FOUND)
#     message(AUTHOR_WARNING,"Open Motion Planning Library not found")
# endif(NOT OMPL_FOUND)

find_package(PkgConfig REQUIRED)
pkg_check_modules(FLANN REQUIRED flann)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
)
include_directories(
  include
  include
)
add_library(${PROJECT_NAME}
  src/planner_core.cpp
  src/toolbox/collisiondetection/costmap_model.cpp
  src/algorithm/algorithm.cpp
  src/algorithm/AStar.cpp
  src/algorithm/HAStar.cpp
  src/base_type/primitive.cpp
  src/algorithm/Voronoi.cpp
  src/toolbox/collisiondetection/collisiondetection.cpp
  src/toolbox/voronoi/dynamicvoronoi.cpp
  src/base_type/node2d.cpp
  src/base_type/pose2d.cpp
  src/planner.cpp
  src/toolbox/voronoi/bucketedqueue.cpp
  src/toolbox/toolbox.cpp
  src/toolbox/visual/visual.cpp
  
  include/planner_core.h
  include/toolbox/collisiondetection/world_model.h
  include/toolbox/collisiondetection/costmap_model.h
  include/toolbox/collisiondetection/line_iterator.h
  include/algorithm/algorithm.h
  include/algorithm/AStar.h
  include/algorithm/HAStar.h
  include/base_type/primitive.h
  include/algorithm/Voronoi.h
  include/toolbox/collisiondetection/collisiondetection.h
  include/constants.h
  include/toolbox/voronoi/dynamicvoronoi.h
  include/toolbox/voronoi/bucketedqueue.h
  include/toolbox/voronoi/point.h
  include/helper.h
  include/base_type/node2d.h
  include/base_type/pose2d.h
  include/planner.h
  include/toolbox/toolbox.h
  include/toolbox/visual/visual.h

)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${FLANN_LIBRARIES})
# target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES})


add_executable(planner
  src/plan_node.cpp 
  src/toolbox/collisiondetection/costmap_model.cpp
  src/algorithm/algorithm.cpp
  src/algorithm/AStar.cpp
  src/algorithm/HAStar.cpp
  src/base_type/primitive.cpp
  src/algorithm/Voronoi.cpp
  src/toolbox/collisiondetection/collisiondetection.cpp
  src/toolbox/voronoi/dynamicvoronoi.cpp
  src/base_type/node2d.cpp
  src/base_type/pose2d.cpp
  src/planner.cpp
  src/toolbox/voronoi/bucketedqueue.cpp
  src/toolbox/toolbox.cpp
  src/toolbox/visual/visual.cpp
)
add_dependencies(planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planner
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# set_target_properties(planner PROPERTIES COMPILE_FLAGS "-g")

install(TARGETS ${PROJECT_NAME} planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

install(FILES bgp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
