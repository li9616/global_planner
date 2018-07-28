#include "algorithm/algorithm.h"
#include <Eigen/Dense>
#include <boost/heap/binomial_heap.hpp>
#include <geometry_msgs/PoseArray.h>
// #include <geometry_msgs/PoseStamped.h>

using namespace yt;

Algorithm::Algorithm(std::string frame_id, double origin_position_x, double origin_position_y, double gridmap_resolution) { 
    ros::NodeHandle private_nh("~");
    frame_id_ = frame_id;
    mid_result_pub_ = private_nh.advertise<geometry_msgs::PoseArray>("mid_result", 1);

    origin_position_x_ = origin_position_x;
    origin_position_y_ = origin_position_y;

    gridmap_resolution_ = gridmap_resolution;


}

void Algorithm::publishMidResult(const std::vector<global_planner::Pose2D>& mid_result ){
    if(mid_result.size() == 0){
        std::cout << "YT: algorithm cannot find midresult" << std::endl;
        return;
    }
    geometry_msgs::PoseArray mid_result_message;
    mid_result_message.poses.resize(mid_result.size());

    for(unsigned int i = 0; i < mid_result.size(); i++){
        mid_result_message.poses.at(i).position.x = mid_result.at(i).getX() * gridmap_resolution_ + origin_position_x_; 
        mid_result_message.poses.at(i).position.y = mid_result.at(i).getY() * gridmap_resolution_ + origin_position_y_;
        tf::Quaternion q = tf::createQuaternionFromYaw(mid_result.at(i).getT());
        tf::quaternionTFToMsg(q, mid_result_message.poses.at(i).orientation);
    }

    mid_result_message.header.frame_id = frame_id_;
    mid_result_message.header.stamp = ros::Time::now();
    mid_result_pub_.publish(mid_result_message);
}