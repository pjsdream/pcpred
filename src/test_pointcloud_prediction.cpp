#include <ros/ros.h>

#include <Eigen/Dense>
#include <vector>

#include <pcpred/pcpred.h>

#include <iostream>

using namespace pcpred;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_pointcloud_prediction");

    ROS_INFO("test_pointcloud_prediction");

    MarkerVisualizer visualizer("pointcloud_prediction_test");

    return 0;
}
