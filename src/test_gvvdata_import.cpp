#include <ros/ros.h>

#include <stdio.h>
#include <pcpred/pcpred.h>

#include <iostream>

using namespace pcpred;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_gvvdata_import");

    ROS_INFO("test_gvvdata_import");

    GVVDataImporter importer;
    Visualizer visualizer("gvvdata");

    ros::Rate rate(33);

    for (int i=0; i<1438; i++)
    {
        printf("frame %d\n", i);
        fflush(stdout);

        importer.import(1, i);
        visualizer.drawPointCloud(importer.pointcloud());

        rate.sleep();
    }

    return 0;
}

