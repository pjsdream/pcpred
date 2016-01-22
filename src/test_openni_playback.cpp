#include <ros/ros.h>

#include <pcpred/import/openni_importer.h>
#include <pcpred/visualization/pointcloud_visualizer.h>

#include <stdio.h>

using namespace pcpred;


int main(int argc, char** argv)
{
    int sequence_number = 1;
    if (argc >= 2)
        sequence_number = atoi(argv[1]);

    ros::init(argc, argv, "test_openni_playback");
    ROS_INFO("test_openni_playback");
    ros::NodeHandle n;

    ros::Rate rate(30);

    OpenniImporter importer;
    PointcloudVisualizer pointcloud_visualizer("openni_test");

    int frame = 0;
    while (importer.import(sequence_number, frame))
    {
        printf("frame %4d", frame);
        fflush(stdout);

        Pointcloud pointcloud( importer.pointcloud() );
        pointcloud_visualizer.drawPointcloud(pointcloud);

        printf("  #points = %5d\n", pointcloud.size());
        fflush(stdout);

        rate.sleep();

        frame++;
    }

    return 0;
}

