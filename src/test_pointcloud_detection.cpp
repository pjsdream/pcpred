#include <ros/ros.h>

#include <pcpred/import/gvv_data_importer.h>
#include <pcpred/detection/depth_frame_human_detector.h>

#include <stdio.h>

using namespace pcpred;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_pointcloud_detection");

    ROS_INFO("test_pointcloud_detection");

    GVVDataImporter importer;
    importer.import(1, 0, false);
    importer.printFrameInfo();

    return 0;
}
