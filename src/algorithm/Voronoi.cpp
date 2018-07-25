#include "algorithm/Voronoi.h"

using namespace yt;

bool findPath(std::vector<std::pair<float, float> > *path,
              int init_x, int init_y,
              int goal_x, int goal_y,
              global_planner::DynamicVoronoi *voronoi,
              bool check_is_voronoi_cell,
              bool stop_at_voronoi );


bool yt::Voronoi::plan(global_planner::Pose2D& start_temp, 
                    const global_planner::Pose2D& goal_temp, 
                    global_planner::Pose2D* nodes3D_temp, 
                    global_planner::Node2D* nodes2D, 
                    int width, 
                    int height, 
                    CollisionDetection* configurationSpace, 
                    global_planner::DynamicVoronoi* voronoiDiagram, 
                    std::vector<global_planner::Pose2D>& plan){



if(!using_voronoi_){
    ROS_ERROR("YT: Voronoi_planner cannot run without voronoi_graph plugin");
    return false;
}


int start_x = (int)start_temp.getX();
int start_y = (int)start_temp.getY();
int goal_x = (int)goal_temp.getX();
int goal_y = (int)goal_temp.getY();

    bool res1 = false, res2 = false, res3 = false;
    //YT find the path from voronoi to goal
    if( !voronoiDiagram->isVoronoi(goal_x,goal_y) )
    {
        //        path3 = findPath( goal, init, A, 0, 1 );
        res3 = findPath( &path3_, goal_x, goal_y, start_x, start_y, voronoiDiagram, 0, 1 );
        std::cout << "findPath 3 res " << res3 << std::endl;
        //        goal = path3(end,:);
        goal_x = std::get<0>( path3_[path3_.size()-1] );
        goal_y = std::get<1>( path3_[path3_.size()-1] );

        std::cout << "voronoi.isVoronoi(goal_x,goal_y) " << voronoiDiagram->isVoronoi(goal_x,goal_y) << std::endl;

        std::reverse(path3_.begin(), path3_.end());
    }

    //YT find the path from start to voronoi
    if( !voronoiDiagram->isVoronoi(start_x,start_y) )
    {
        res1 = findPath( &path1_, start_x, start_y, goal_x, goal_y, voronoiDiagram, 0, 1 );
        std::cout << "findPath 1 res " << res1 << std::endl;
        start_x = std::get<0>( path1_[path1_.size()-1] );
        start_y = std::get<1>( path1_[path1_.size()-1] );

        std::cout << "voronoi.isVoronoi(start_x,start_y) " << voronoiDiagram->isVoronoi(start_x,start_y) << std::endl;
    }

    //YT find the path on voronoi
    res2 = findPath( &path2_, start_x, start_y, goal_x, goal_y, voronoiDiagram, 1, 0 );
    std::cout << "findPath 2 res " << res2 << std::endl;


    if(!(res1 && res2 && res3))
    {
        ROS_INFO("Failed to find full path");
        return false;
    }

    path1_.insert( path1_.end(), path2_.begin(), path2_.end() );
    path1_.insert( path1_.end(), path3_.begin(), path3_.end() );


std::cout << "YT: check!" << std::endl;
    for(int i = path1_.size() - 1; i >= 0; i--)
    {
        global_planner::Pose2D pose;

        pose.setX(std::get<0>(path1_[i]));
        pose.setY(std::get<1>(path1_[i]));

        plan.push_back( pose );

    }
    std::cout << "YT: 2" << std::endl;
    return !plan.empty();
}

bool findPath(std::vector<std::pair<float, float> > *path,
              int init_x, int init_y,
              int goal_x, int goal_y,
              global_planner::DynamicVoronoi *voronoi,
              bool check_is_voronoi_cell,
              bool stop_at_voronoi )
{
//    ROS_INFO("init_x %d, init_y %d, goal_x %d, goal_y %d, check_is_voronoi_cell %d, stop_at_voronoi %d", init_x, init_y, goal_x, goal_y, check_is_voronoi_cell, stop_at_voronoi);
//    ROS_INFO("isVoronoi(init) %d; isVoronoi(goal) %d", voronoi->isVoronoi(init_x, init_y), voronoi->isVoronoi(goal_x, goal_y) );
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

    for (unsigned int y = sizeY-1; y>=0; y--) {
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
    for (unsigned int y=sizeY-1; y>=0; y--) {
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
