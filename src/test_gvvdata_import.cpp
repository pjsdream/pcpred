#include <ros/ros.h>

#include <stdio.h>
#include <pcpred/pcpred.h>

#include <iostream>

#include <pcl/console/print.h>

using namespace pcpred;


int main(int argc, char** argv)
{
    int sequence_number = 1;
    if (argc >= 2)
        sequence_number = atoi(argv[1]);

    ros::init(argc, argv, "test_gvvdata_import");
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    ROS_INFO("test_gvvdata_import");

    GVVDataImporter importer;
    PointcloudVisualizer pointcloud_visualizer("gvvdata_test");
    PointcloudVisualizer cluster_visualizer("gvvdata_cluster");

    ros::Rate rate(33);

    int frame = 0;
    while (importer.import(sequence_number, frame, true))
    {
        printf("frame %4d", frame);
        fflush(stdout);

        Pointcloud pointcloud( importer.pointcloud() );
        pointcloud.rotate(M_PI / 2.0, Eigen::Vector3d(1, 0, 0));
        pointcloud_visualizer.drawPointcloud(pointcloud);

        Pointcloud clusters = pointcloud.cluster(0.1, 0.2);
        cluster_visualizer.drawPointcloud(clusters);

        printf("  #points = %5d  #clusters = %5d\n", pointcloud.size(), clusters.size());
        fflush(stdout);

        rate.sleep();

        frame++;
    }

    return 0;
}

