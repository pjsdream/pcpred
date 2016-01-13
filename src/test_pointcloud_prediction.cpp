#include <ros/ros.h>

#include <Eigen/Dense>
#include <vector>

#include <pcpred/pcpred.h>

#include <iostream>

using namespace pcpred;


int main(int argc, char** argv)
{
    int sequence_number = 1;
    if (argc >= 2)
        sequence_number = atoi(argv[1]);

    ros::init(argc, argv, "test_pointcloud_prediction");

    ROS_INFO("test_pointcloud_prediction");

    MarkerVisualizer visualizer("pointcloud_prediction_test");
    PointcloudVisualizer pointcloud_visualizer("pointcloud_test");

    GVVDataImporter importer;

    ros::Rate rate(33);

    int frame = 0;
    while (importer.import(sequence_number, frame, true))
    {
        printf("frame %4d", frame);

        Pointcloud pointcloud( importer.pointcloud() );
        pointcloud.rotate(M_PI / 2.0, Eigen::Vector3d(1, 0, 0));
        pointcloud_visualizer.drawPointcloud(pointcloud);

        printf("  #points = %5d", pointcloud.size());
        fflush(stdout);

        rate.sleep();

        frame++;
    }

    return 0;
}
