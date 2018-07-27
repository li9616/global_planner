#include "toolbox/voronoi/dynamicvoronoi.h"
#include "base_type/node2d.h"
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <utility>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <string>

using namespace global_planner;

DynamicVoronoi::DynamicVoronoi(costmap_2d::Costmap2D* costmap, int cell_divider, std::string frame_id):
Plugin(costmap, cell_divider, frame_id)
{
  
  ros::NodeHandle private_nh("~");

  generalized_voronoi_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("GVD/map", 1);

  sqrt2 = sqrt(2.0);
  data = NULL;
  gridMap = NULL;
  map_for_show_ = NULL;
}

DynamicVoronoi::~DynamicVoronoi() {
  if (data) {
    for (int x=0; x<sizeX; x++) delete[] data[x];
    delete[] data;
  }
}

void DynamicVoronoi::initializeEmpty(int _sizeX, int _sizeY, bool initGridMap) {
  sizeX = _sizeX;
  sizeY = _sizeY;
  if (data) {
    for (int x=0; x<sizeX; x++) delete[] data[x];
    delete[] data;
  }
  data = new dataCell*[sizeX];
  for (int x=0; x<sizeX; x++) data[x] = new dataCell[sizeY];

  if (initGridMap) {
    if (gridMap) {
      for (int x=0; x<sizeX; x++) delete[] gridMap[x];
      delete[] gridMap;
    }
    gridMap = new bool*[sizeX];
    for (int x=0; x<sizeX; x++) gridMap[x] = new bool[sizeY];
  }
  
  dataCell c;
  c.dist = INFINITY;
  c.sqdist = INT_MAX;
  c.obstX = invalidObstData;
  c.obstY = invalidObstData;
  c.voronoi = free;
  c.queueing = fwNotQueued;
  c.needsRaise = false;

  for (int x=0; x<sizeX; x++)
    for (int y=0; y<sizeY; y++) data[x][y] = c;

  if (initGridMap) {
    for (int x=0; x<sizeX; x++) 
      for (int y=0; y<sizeY; y++) gridMap[x][y] = 0;
  }

  //YT 初始化map_for_show的内存，visualize函数中赋值，通过get函数访问
  if(!map_for_show_){
    map_for_show_ = new unsigned char[sizeX * sizeY];
  }
}

void DynamicVoronoi::initializeMap(int _sizeX, int _sizeY, bool** _gridMap) {
  gridMap = _gridMap;
  initializeEmpty(_sizeX, _sizeY, false);

  for (int x=0; x<sizeX; x++) {
    for (int y=0; y<sizeY; y++) {
      if (gridMap[x][y]) {
        dataCell c = data[x][y];
        if (!isOccupied(x,y,c)) {
          
          bool isSurrounded = true;
          for (int dx=-1; dx<=1; dx++) {
            int nx = x+dx;
            if (nx<=0 || nx>=sizeX-1) continue;
            for (int dy=-1; dy<=1; dy++) {
              if (dx==0 && dy==0) continue;
              int ny = y+dy;
              if (ny<=0 || ny>=sizeY-1) continue;

              if (!gridMap[nx][ny]) {
                isSurrounded = false;
                break;
              }
            }
          }
          if (isSurrounded) {
            c.obstX = x;
            c.obstY = y;
            c.sqdist = 0;
            c.dist=0;
            c.voronoi=occupied;
            c.queueing = fwProcessed;
            data[x][y] = c;
          } else setObstacle(x,y);
        }
      }
    }
  }
}

void DynamicVoronoi::occupyCell(int x, int y) {
  gridMap[x][y] = 1;
  setObstacle(x,y);
}
void DynamicVoronoi::clearCell(int x, int y) {
  gridMap[x][y] = 0;
  removeObstacle(x,y);
}

void DynamicVoronoi::setObstacle(int x, int y) {
  dataCell c = data[x][y];
  if(isOccupied(x,y,c)) return;
  
  addList.push_back(INTPOINT(x,y));
  c.obstX = x;
  c.obstY = y;
  data[x][y] = c;
}

void DynamicVoronoi::removeObstacle(int x, int y) {
  dataCell c = data[x][y];
  if(isOccupied(x,y,c) == false) return;

  removeList.push_back(INTPOINT(x,y));
  c.obstX = invalidObstData;
  c.obstY  = invalidObstData;    
  c.queueing = bwQueued;
  data[x][y] = c;
}

void DynamicVoronoi::exchangeObstacles(std::vector<INTPOINT> points) {

  for (unsigned int i=0; i<lastObstacles.size(); i++) {
    int x = lastObstacles[i].x;
    int y = lastObstacles[i].y;

    bool v = gridMap[x][y];
    if (v) continue;
    removeObstacle(x,y);
  }  

  lastObstacles.clear();

  for (unsigned int i=0; i<points.size(); i++) {
    int x = points[i].x;
    int y = points[i].y;
    bool v = gridMap[x][y];
    if (v) continue;
    setObstacle(x,y);
    lastObstacles.push_back(points[i]);
  }  
}

void DynamicVoronoi::update(bool updateRealDist) {

  commitAndColorize(updateRealDist);

  while (!open.empty()) {
    INTPOINT p = open.pop();
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if(c.queueing==fwProcessed) continue; 

    if (c.needsRaise) {
      // RAISE
      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue;
          dataCell nc = data[nx][ny];
          if (nc.obstX!=invalidObstData && !nc.needsRaise) {
            if(!isOccupied(nc.obstX,nc.obstY,data[nc.obstX][nc.obstY])) {
              open.push(nc.sqdist, INTPOINT(nx,ny));
              nc.queueing = fwQueued;
              nc.needsRaise = true;
              nc.obstX = invalidObstData;
              nc.obstY = invalidObstData;
              if (updateRealDist) nc.dist = INFINITY;
              nc.sqdist = INT_MAX;
              data[nx][ny] = nc;
            } else {
              if(nc.queueing != fwQueued){
                open.push(nc.sqdist, INTPOINT(nx,ny));
                nc.queueing = fwQueued;
                data[nx][ny] = nc;
              }
            }      
          }
        }
      }
      c.needsRaise = false;
      c.queueing = bwProcessed;
      data[x][y] = c;
    }
    else if (c.obstX != invalidObstData && isOccupied(c.obstX,c.obstY,data[c.obstX][c.obstY])) {

      // LOWER
      c.queueing = fwProcessed;
      c.voronoi = occupied;

      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue;
          dataCell nc = data[nx][ny];
          if(!nc.needsRaise) {
            int distx = nx-c.obstX;
            int disty = ny-c.obstY;
            int newSqDistance = distx*distx + disty*disty;		
            bool overwrite =  (newSqDistance < nc.sqdist);
            if(!overwrite && newSqDistance==nc.sqdist) { 
              if (nc.obstX == invalidObstData || isOccupied(nc.obstX,nc.obstY,data[nc.obstX][nc.obstY])==false) overwrite = true;
            }
            if (overwrite) {
              open.push(newSqDistance, INTPOINT(nx,ny));
              nc.queueing = fwQueued;
              if (updateRealDist) {
                nc.dist = sqrt((double) newSqDistance);
              }
              nc.sqdist = newSqDistance;
              nc.obstX = c.obstX;
              nc.obstY = c.obstY;
            } else { 
              checkVoro(x,y,nx,ny,c,nc);
            }
            data[nx][ny] = nc;
          }
        }
      }
    }
    data[x][y] = c;
  }
}

float DynamicVoronoi::getDistance( int x, int y ) {
  if( (x>0) && (x<sizeX) && (y>0) && (y<sizeY)) return data[x][y].dist; 
  else return -INFINITY;
}

bool DynamicVoronoi::isVoronoi( int x, int y ) {
  dataCell c = data[x][y];
  return (c.voronoi==free || c.voronoi==voronoiKeep);
}


void DynamicVoronoi::commitAndColorize(bool updateRealDist) {
  // ADD NEW OBSTACLES
  for (unsigned int i=0; i<addList.size(); i++) {
    INTPOINT p = addList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if(c.queueing != fwQueued){
      if (updateRealDist) c.dist = 0;
      c.sqdist = 0;
      c.obstX = x;
      c.obstY = y;
      c.queueing = fwQueued;
      c.voronoi = occupied;
      data[x][y] = c;
      open.push(0, INTPOINT(x,y));
    }
  }

  // REMOVE OLD OBSTACLES
  for (unsigned int i=0; i<removeList.size(); i++) {
    INTPOINT p = removeList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if (isOccupied(x,y,c)==true) continue; // obstacle was removed and reinserted
    open.push(0, INTPOINT(x,y));
    if (updateRealDist) c.dist  = INFINITY;
    c.sqdist = INT_MAX;
    c.needsRaise = true;
    data[x][y] = c;
  }
  removeList.clear();
  addList.clear();
}


void DynamicVoronoi::checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc) {

  if ((c.sqdist>1 || nc.sqdist>1) && nc.obstX!=invalidObstData) { 
    if (abs(c.obstX-nc.obstX) > 1 || abs(c.obstY-nc.obstY) > 1) {
      //compute dist from x,y to obstacle of nx,ny	 
      int dxy_x = x-nc.obstX;
      int dxy_y = y-nc.obstY;
      int sqdxy = dxy_x*dxy_x + dxy_y*dxy_y;
      int stability_xy = sqdxy - c.sqdist;
      if (sqdxy - c.sqdist<0) return;

      //compute dist from nx,ny to obstacle of x,y
      int dnxy_x = nx - c.obstX;
      int dnxy_y = ny - c.obstY;
      int sqdnxy = dnxy_x*dnxy_x + dnxy_y*dnxy_y;
      int stability_nxy = sqdnxy - nc.sqdist;
      if (sqdnxy - nc.sqdist <0) return;

      //which cell is added to the Voronoi diagram?
      if(stability_xy <= stability_nxy && c.sqdist>2) {
        if (c.voronoi != free) {
          c.voronoi = free;
          reviveVoroNeighbors(x,y);
          pruneQueue.push(INTPOINT(x,y));
        }
      }
      if(stability_nxy <= stability_xy && nc.sqdist>2) {
        if (nc.voronoi != free) {
          nc.voronoi = free;
          reviveVoroNeighbors(nx,ny);
          pruneQueue.push(INTPOINT(nx,ny));
        }
      }
    }
  }
}


void DynamicVoronoi::reviveVoroNeighbors(int &x, int &y) {
  for (int dx=-1; dx<=1; dx++) {
    int nx = x+dx;
    if (nx<=0 || nx>=sizeX-1) continue;
    for (int dy=-1; dy<=1; dy++) {
      if (dx==0 && dy==0) continue;
      int ny = y+dy;
      if (ny<=0 || ny>=sizeY-1) continue;
      dataCell nc = data[nx][ny];
      if (nc.sqdist != INT_MAX && !nc.needsRaise && (nc.voronoi == voronoiKeep || nc.voronoi == voronoiPrune)) {
        nc.voronoi = free;
        data[nx][ny] = nc;
        pruneQueue.push(INTPOINT(nx,ny));
      }
    }
  }
}


bool DynamicVoronoi::isOccupied(int x, int y) {
  dataCell c = data[x][y];
  return (c.obstX==x && c.obstY==y);
}

bool DynamicVoronoi::isOccupied(int &x, int &y, dataCell &c) { 
  return (c.obstX==x && c.obstY==y);
}

void DynamicVoronoi::visualize(const char *filename) {
  // write pgm files
voronoi_point_.clear();

  FILE* F = fopen(filename, "w");
  if (!F) {
    std::cerr << "could not open 'result.pgm' for writing!\n";
    return;
  }
  fprintf(F, "P6\n");
  fprintf(F, "%d %d 255\n", sizeX, sizeY);

  for(int y = sizeY-1; y >=0; y--){      
    for(int x = 0; x<sizeX; x++){	
      unsigned char c = 0;
      if (isVoronoi(x,y)) {
        fputc( 255, F );
        fputc( 0, F );
        fputc( 0, F );
        *(map_for_show_ + y * sizeX + x) = 128;
        voronoi_point_.push_back(std::pair<int, int>(x, y));
      } else if (data[x][y].sqdist==0) {
        fputc( 0, F );
        fputc( 0, F );
        fputc( 0, F );
        *(map_for_show_ + y * sizeX + x) = 0;
      } else {
        float f = 80+(data[x][y].dist*5);
        if (f>255) f=255;
        if (f<0) f=0;
        c = (unsigned char)f;
        fputc( c, F );
        fputc( c, F );
        fputc( c, F );
        *(map_for_show_ + y * sizeX + x) = c;
      }
    }
  }
  std::cout << "YT: check the size of voronoi_point_: " << voronoi_point_.size() << std::endl;
  fclose(F);
  publishGeneralizedVoronoi();
}



void DynamicVoronoi::prune() {
  // filler
  while(!pruneQueue.empty()) {
    INTPOINT p = pruneQueue.front();
    pruneQueue.pop();
    int x = p.x;
    int y = p.y;

    if (data[x][y].voronoi==occupied) continue;
    if (data[x][y].voronoi==freeQueued) continue;

    data[x][y].voronoi = freeQueued;
    open.push(data[x][y].sqdist, p);

    /* tl t tr
       l c r
       bl b br */

    dataCell tr,tl,br,bl;
    tr = data[x+1][y+1];
    tl = data[x-1][y+1];
    br = data[x+1][y-1];
    bl = data[x-1][y-1];

    dataCell r,b,t,l;
    r = data[x+1][y];
    l = data[x-1][y];
    t = data[x][y+1];
    b = data[x][y-1];

    if (x+2<sizeX && r.voronoi==occupied) { 
      // fill to the right
      if (tr.voronoi!=occupied && br.voronoi!=occupied && data[x+2][y].voronoi!=occupied) {
        r.voronoi = freeQueued;
        open.push(r.sqdist, INTPOINT(x+1,y));
        data[x+1][y] = r;
      }
    } 
    if (x-2>=0 && l.voronoi==occupied) { 
      // fill to the left
      if (tl.voronoi!=occupied && bl.voronoi!=occupied && data[x-2][y].voronoi!=occupied) {
        l.voronoi = freeQueued;
        open.push(l.sqdist, INTPOINT(x-1,y));
        data[x-1][y] = l;
      }
    } 
    if (y+2<sizeY && t.voronoi==occupied) { 
      // fill to the top
      if (tr.voronoi!=occupied && tl.voronoi!=occupied && data[x][y+2].voronoi!=occupied) {
        t.voronoi = freeQueued;
        open.push(t.sqdist, INTPOINT(x,y+1));
        data[x][y+1] = t;
      }
    } 
    if (y-2>=0 && b.voronoi==occupied) { 
      // fill to the bottom
      if (br.voronoi!=occupied && bl.voronoi!=occupied && data[x][y-2].voronoi!=occupied) {
        b.voronoi = freeQueued;
        open.push(b.sqdist, INTPOINT(x,y-1));
        data[x][y-1] = b;
      }
    } 
  }


  while(!open.empty()) {
    INTPOINT p = open.pop();
    dataCell c = data[p.x][p.y];
    int v = c.voronoi;
    if (v!=freeQueued && v!=voronoiRetry) { // || v>free || v==voronoiPrune || v==voronoiKeep) {
      //      assert(v!=retry);
      continue;
    }

    markerMatchResult r = markerMatch(p.x,p.y);
    if (r==pruned) c.voronoi = voronoiPrune;
    else if (r==keep) c.voronoi = voronoiKeep;
    else { // r==retry
      c.voronoi = voronoiRetry;
      //      printf("RETRY %d %d\n", x, sizeY-1-y);
      pruneQueue.push(p);
    }
    data[p.x][p.y] = c;

    if (open.empty()) {
      while (!pruneQueue.empty()) {
        INTPOINT p = pruneQueue.front();
        pruneQueue.pop();
        open.push(data[p.x][p.y].sqdist, p);
      }
    }
  }
  //  printf("match: %d\nnomat: %d\n", matchCount, noMatchCount);
}


DynamicVoronoi::markerMatchResult DynamicVoronoi::markerMatch(int x, int y) {
  // implementation of connectivity patterns
  bool f[8];

  int nx, ny;
  int dx, dy;

  int i=0;
  int count=0;
  //  int obstacleCount=0;
  int voroCount=0;
  int voroCountFour=0;

  for (dy=1; dy>=-1; dy--) {
    ny = y+dy;
    for (dx=-1; dx<=1; dx++) {
      if (dx || dy) {
        nx = x+dx;
        dataCell nc = data[nx][ny];
        int v = nc.voronoi;
        bool b = (v<=free && v!=voronoiPrune); 
        //	if (v==occupied) obstacleCount++;
        f[i] = b;
        if (b) {
          voroCount++;
          if (!(dx && dy)) voroCountFour++;
        }
        if (b && !(dx && dy) ) count++;
        //	if (v<=free && !(dx && dy)) voroCount++;
        i++;
      }
    }
  }
  if (voroCount<3 && voroCountFour==1 && (f[1] || f[3] || f[4] || f[6])) {
    //    assert(voroCount<2);
    //    if (voroCount>=2) printf("voro>2 %d %d\n", x, y);
    return keep;
  }

  // 4-connected
  if ((!f[0] && f[1] && f[3]) || (!f[2] && f[1] && f[4]) || (!f[5] && f[3] && f[6]) || (!f[7] && f[6] && f[4])) return keep;
  if ((f[3] && f[4] && !f[1] && !f[6]) || (f[1] && f[6] && !f[3] && !f[4])) return keep;
  


  // keep voro cells inside of blocks and retry later
  if (voroCount>=5 && voroCountFour>=3 && data[x][y].voronoi!=voronoiRetry) {
    return retry;
  }

  return pruned;
}


bool DynamicVoronoi::getPathInVoronoi(int start_x, int start_y, int goal_x, int goal_y, std::vector<global_planner::Pose2D>& plan) {//YT based on cell of gridmap

  // std::vector<yt::Pose2D> plan;

    bool res1 = false, res2 = false, res3 = false;
    //YT find the path from voronoi to goal
    if( !isVoronoi(goal_x,goal_y) )
    {
        res3 = findPath( &path3_, goal_x, goal_y, start_x, start_y, this, 0, 1 );
        // std::cout << "findPath 3 res " << res3 << std::endl;
        //        goal = path3(end,:);
        goal_x = std::get<0>( path3_[path3_.size()-1] );
        goal_y = std::get<1>( path3_[path3_.size()-1] );

        std::cout << "voronoi.isVoronoi(goal_x,goal_y) " << isVoronoi(goal_x,goal_y) << std::endl;

        std::reverse(path3_.begin(), path3_.end());
    }

    //YT find the path from start to voronoi
    if( !isVoronoi(start_x,start_y) )
    {
        res1 = findPath( &path1_, start_x, start_y, goal_x, goal_y, this, 0, 1 );
        // std::cout << "findPath 1 res " << res1 << std::endl;
        start_x = std::get<0>( path1_[path1_.size()-1] );
        start_y = std::get<1>( path1_[path1_.size()-1] );

        std::cout << "voronoi.isVoronoi(start_x,start_y) " << isVoronoi(start_x,start_y) << std::endl;
    }

    //YT find the path on voronoi
    res2 = findPath( &path2_, start_x, start_y, goal_x, goal_y, this, 1, 0 );
    // std::cout << "findPath 2 res " << res2 << std::endl;


    if(!(res1 && res2 && res3))
    {
        ROS_INFO("Failed to find full path");
        return false;
    }

    path1_.insert( path1_.end(), path2_.begin(), path2_.end() );
    path1_.insert( path1_.end(), path3_.begin(), path3_.end() );

    // std::cout << "YT: check!" << std::endl;
    for(int i = path1_.size() - 1; i >= 0; i--)
    {
        global_planner::Pose2D pose;

        pose.setX(std::get<0>(path1_[i]));
        pose.setY(std::get<1>(path1_[i]));

        plan.push_back( pose );

    }

    return !plan.empty();
}




bool DynamicVoronoi::findPath(std::vector<std::pair<float, float> > *path,
              int init_x, int init_y,
              int goal_x, int goal_y,
              global_planner::DynamicVoronoi *voronoi,
              bool check_is_voronoi_cell,
              bool stop_at_voronoi )
{
    // available movements (actions) of the robot on the grid
    std::vector<std::pair<int, int> > delta;
    delta.push_back( {-1, 0} );     // go up
    delta.push_back( {0, -1} );     // go left
    delta.push_back( {1, 0} );      // go down
    delta.push_back( {0, 1} );      // go right
    delta.push_back( {-1, -1} );    // up and left
    delta.push_back( {-1, 1} );     // up and right
    delta.push_back( {1, -1} );     // down and left
    delta.push_back( {1,  1} );     // down and right

    // cost of movement
    float cost = 1;

    // grid size
    unsigned int sizeX = voronoi->getSizeX();
    unsigned int sizeY = voronoi->getSizeY();

    // closed cells grid (same size as map grid)
    bool **closed=NULL;
    closed = new bool*[sizeX];
    for (unsigned int x=0; x<sizeX; x++) {
        (closed)[x] = new bool[sizeY];
    }

    for (int y = sizeY-1; y>=0; y--) {
        for (unsigned int x=0; x<sizeX; x++) {
            (closed)[x][y] = false;
        }
    }

    // actions (number of delta's row) cells grid (same size as map grid)
    int **action=NULL;
    action = new int*[sizeX];
    for (unsigned int x=0; x<sizeX; x++) {
        (action)[x] = new int[sizeY];
    }
    for (int y=sizeY-1; y>=0; y--) {
        for (unsigned int x=0; x<sizeX; x++) {
            (action)[x][y] = -1;
        }
    }

    // set current cell
    int x = init_x;
    int y = init_y;

    // set cost
    float g = 0;

    //f = heuristic(x,y) + g;

    // vector of open (for possible expansion) nodes
    std::vector<std::tuple<float, int, int> > open;
    open.push_back( std::make_tuple( g, x, y ) );

    // path found flag
    bool found = false;
    // no solution could be found flag
    bool resign = false;

    while( !found && !resign )
    {
        if (open.size() == 0)
        {
            resign = true;
            path->empty();
            return false;
        }
        else
        {
            // sort open by cost
            sort(open.begin(), open.end());
            reverse(open.begin(), open.end());
            // get node with lowest cost
            std::tuple<float, int, int> next = open[open.size()-1];
            open.pop_back();
            g = std::get<0>(next);
            x = std::get<1>(next);
            y = std::get<2>(next);

            // check, whether the solution is found (we are at the goal)
            if(stop_at_voronoi)
            {
                // if stop_at_voronoi is set, we stop, when get path to any voronoi cell
                if(voronoi->isVoronoi(x,y))
                {
                    found = true;
                    goal_x = x;
                    goal_y = y;
                    continue;
                }
            }
            else
            {
                if ( x == goal_x && y == goal_y )
                {
                    found = true;
                    continue;
                }
            }
            for( unsigned int i=0; i < delta.size(); i++ )
            {
                // expansion
                int x2 = x + std::get<0>(delta[i]);
                int y2 = y + std::get<1>(delta[i]);

                // check new node to be in grid bounds
                if ( x2 >= 0 && x2 < sizeX && y2 >= 0 && y2 < sizeY )
                {
                    // check new node not to be in obstacle
                    if(voronoi->isOccupied(x2,y2))
                    {
                        continue;
                    }
                    // check new node was not early visited
                    if ( closed[x2][y2] ){
                        continue;
                    }

                    // check new node is on Voronoi diagram
                    if (!voronoi->isVoronoi(x2,y2) && check_is_voronoi_cell){
                        continue;
                    }

                    float g2 = g + cost;
                    //                        f2 = heuristic(x2,y2) + g2;
                    open.push_back( std::make_tuple( g2, x2, y2 ) );
                    closed[x2][y2] = true;
                    action[x2][y2] = i;
                }
            }
        }
    }

    // Make reverse steps from goal to init to write path
    x = goal_x;
    y = goal_y;

    int i = 0;
    path->clear();

    while( x != init_x || y != init_y )
    {
        path->push_back({x,y});
        i++;

        int x2 = x - std::get<0>( delta[ action[x][y] ] );
        int y2 = y - std::get<1>( delta[ action[x][y] ] );

        x = x2;
        y = y2;
    }

    reverse(path->begin(), path->end());
    return true;
}

// global_planner::Pose2D DynamicVoronoi::getNearestPointOnVor(int cell_x, int cell_y){
//   global_planner::Pose2D pose;//YT only use x and y 

//   //YT get all the point on voronoi_path
//   voronoi_point_.clear();
//     unsigned int sizeY = getSizeY();
//         unsigned int sizeX = getSizeX();
//   // for(int y = sizeY-1; y >=0; y--){      
//   //   for(int x = 0; x<sizeX; x++){	
//   //     unsigned char c = 0;
//   //     if (isVoronoi(x,y)) {
//   //       voronoi_point_.push_back(std::pair<int, int>(x, y));
//   //     }
//   //   }
//   // }
//   std::cout << "YT: check the size of voronoi_point_: "<< voronoi_point_.size() << std::endl;


// //YT setup KDTree


// //YT search


//   return pose;
// }




void DynamicVoronoi::publishGeneralizedVoronoi()
{
        nav_msgs::OccupancyGrid map_temp;
        map_temp.header.stamp = ros::Time::now();
        map_temp.header.frame_id = frame_id_;//YT 这里等参数可以传进来了就改用frame_id_;
        map_temp.info.resolution = costmap_->getResolution() / cell_divider_;
        map_temp.info.width = costmap_->getSizeInCellsX() * cell_divider_;
        map_temp.info.height = costmap_->getSizeInCellsY() * cell_divider_;
        map_temp.info.origin.position.x = costmap_->getOriginX();
        map_temp.info.origin.position.y = costmap_->getOriginY();
        map_temp.info.origin.position.z = 0;
        map_temp.info.origin.orientation.x = 0;
        map_temp.info.origin.orientation.y = 0;
        map_temp.info.origin.orientation.z = 0;
        map_temp.info.origin.orientation.w = 1;

        unsigned char* vor_map = NULL;

        vor_map = getMapForShow();
        //bug,如果没有路径结果会返回空指针
        if(vor_map != NULL){
            std::cout << "YT: prepare to copy voronoi_graph" << std::endl;
            map_temp.data.resize(map_temp.info.width * map_temp.info.height);
            memcpy((void*)map_temp.data.data(), (void*)vor_map, sizeof(unsigned char) * map_temp.info.width * map_temp.info.height);
            std::cout << "YT: copy voronoi_graph finished" << std::endl;
            generalized_voronoi_pub_.publish(map_temp);
        }
  
}

