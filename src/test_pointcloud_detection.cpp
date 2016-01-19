#include <ros/ros.h>

#include <pcpred/import/gvv_data_importer.h>
#include <pcpred/detection/pointcloud_human_detector.h>
#include <pcpred/visualization/pointcloud_visualizer.h>

#include <stdio.h>

using namespace pcpred;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_pointcloud_detection");
    ros::NodeHandle n;
    ros::Rate rate(33);

    ROS_INFO("test_pointcloud_detection");

    GVVDataImporter importer;

    PointcloudHumanDetector detector;
    detector.setVisualizerTopic("pointcloud_human_test");

    PointcloudVisualizer pointcloud_visualizer("pointcloud_test");

    int frame = 0;
    while (importer.import(1, frame, false))
    {
        Pointcloud pointcloud( importer.pointcloud() );
        pointcloud.rotate(M_PI / 2.0, Eigen::Vector3d(1, 0, 0));

        detector.observe(pointcloud);

        pointcloud_visualizer.drawPointcloud(pointcloud);
        detector.visualizeHuman();

        printf("Frame = %4d  points = %5d\n", frame, pointcloud.size());
        fflush(stdout);

        rate.sleep();

        frame++;
    }

    return 0;
}
