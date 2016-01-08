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
    PointcloudVisualizer visualizer("gvvdata_test");

    ros::Rate rate(33);

    for (int i=0; i<1438; i++)
    {
        printf("frame %4d", i);
        fflush(stdout);

        importer.import(1, i);
        Pointcloud pointcloud( importer.pointcloud() );
        pointcloud.rotate(M_PI / 2.0, Eigen::Vector3d(1, 0, 0));
        visualizer.drawPointcloud(pointcloud);

        printf("  #points = %d\n", pointcloud.size());
        fflush(stdout);

        pointcloud.cluster(0.01, 0.1);

        rate.sleep();
    }

    return 0;
}

