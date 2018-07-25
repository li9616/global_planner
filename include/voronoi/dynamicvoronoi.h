#ifndef _DYNAMICVORONOI_H_
#define _DYNAMICVORONOI_H_


#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <queue>
#include <flann/flann.hpp>
#include "voronoi/bucketedqueue.h"
#include "pose2d.h"

namespace global_planner {
//! A DynamicVoronoi object computes and updates a distance map and Voronoi diagram.
class DynamicVoronoi {

 public:

  DynamicVoronoi();
  ~DynamicVoronoi();

  //! Initialization with an empty map
  void initializeEmpty(int _sizeX, int _sizeY, bool initGridMap = true);
  //! Initialization with a given binary map (false==free, true==occupied)
  void initializeMap(int _sizeX, int _sizeY, bool** _gridMap);

  //! add an obstacle at the specified cell coordinate
  void occupyCell(int x, int y);
  //! remove an obstacle at the specified cell coordinate
  void clearCell(int x, int y);
  //! remove old dynamic obstacles and add the new ones
  void exchangeObstacles(std::vector<INTPOINT> newObstacles);

  //! update distance map and Voronoi diagram to reflect the changes
  void update(bool updateRealDist = true);
  //! prune the Voronoi diagram
  void prune();

  //! returns the obstacle distance at the specified location
  float getDistance(int x, int y);
  //! returns whether the specified cell is part of the (pruned) Voronoi graph
  bool isVoronoi(int x, int y);
  //! checks whether the specficied location is occupied
  bool isOccupied(int x, int y);
  //! write the current distance map and voronoi diagram as ppm file
  void visualize(const char* filename = "/home/yangtong/dynamic_voronoi_result.ppm");

  //! returns the horizontal size of the workspace/map
  unsigned int getSizeX() {return sizeX;}
  //! returns the vertical size of the workspace/map
  unsigned int getSizeY() {return sizeY;}

  // bool** get_Map_in_voronoi(){return gridMap;}
  unsigned char* getMapForShow(){return map_for_show_;}

  bool getPathInVoronoi(int start_x, int start_y, int goal_x, int goal_y, std::vector<global_planner::Pose2D>& plan);//YT based on cell of gridmap



  // was private, changed to public for obstX, obstY
 public:
  struct dataCell {
    float dist;
    char voronoi;
    char queueing;
    int obstX;
    int obstY;
    bool needsRaise;
    int sqdist;
  };

  typedef enum {voronoiKeep = -4, freeQueued = -3, voronoiRetry = -2, voronoiPrune = -1, free = 0, occupied = 1} State;
  typedef enum {fwNotQueued = 1, fwQueued = 2, fwProcessed = 3, bwQueued = 4, bwProcessed = 1} QueueingState;
  typedef enum {invalidObstData = SHRT_MAX / 2} ObstDataState;
  typedef enum {pruned, keep, retry} markerMatchResult;



  // methods
  void setObstacle(int x, int y);
  void removeObstacle(int x, int y);
  inline void checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc);
  void recheckVoro();
  void commitAndColorize(bool updateRealDist = true);
  inline void reviveVoroNeighbors(int& x, int& y);

  inline bool isOccupied(int& x, int& y, dataCell& c);
  inline markerMatchResult markerMatch(int x, int y);

  // queues
  BucketPrioQueue open;
  std::queue<INTPOINT> pruneQueue;

  std::vector<INTPOINT> removeList;
  std::vector<INTPOINT> addList;
  std::vector<INTPOINT> lastObstacles;

  // maps
  int sizeY;
  int sizeX;
  dataCell** data;
  bool** gridMap;

  unsigned char* map_for_show_;
  // parameters
  int padding;
  double doubleThreshold;

  double sqrt2;

  // dataCell** getData(){ return data; }

private:
    std::vector<std::pair<float, float> > path1_;
    std::vector<std::pair<float, float> > path2_;
    std::vector<std::pair<float, float> > path3_;

    bool findPath(std::vector<std::pair<float, float> > *path,
              int init_x, int init_y,
              int goal_x, int goal_y,
              global_planner::DynamicVoronoi *voronoi,
              bool check_is_voronoi_cell,
              bool stop_at_voronoi );


};
}

#endif

