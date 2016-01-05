#include <ros/ros.h>

#include <pcpred/pcpred.h>

using namespace pcpred;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "app_pointcloud_prediction");

    ROS_INFO("app_pointcloud_prediction");

    PointcloudPredictor predictor;

    return 0;
}

