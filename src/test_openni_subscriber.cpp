#include <ros/ros.h>

#include <pcpred/import/openni_subscriber.h>
#include <pcpred/visualization/pointcloud_visualizer.h>

#include <stdio.h>

#include <unistd.h>
#include <stdlib.h>

using namespace pcpred;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_openni_subscriber");
    ROS_INFO("test_openni_subscriber");
    ros::NodeHandle n;

    ros::Rate rate(30);

    OpenniSubscriber* subscriber = OpenniSubscriber::getInstance();
    //subscriber->setPrintMessages(true);

    PointcloudVisualizer visualizer("openni_test");

    if (fork() != 0)
    {
        printf("rosbag play started\n");
        fflush(stdout);
        system("rosbag play ../data/bag/kinect1.bag >/dev/null");
        return 0;
    }

    int frame = 0;
    while (true)
    {
        subscriber->readDepthFrame();

        visualizer.drawPointcloud( subscriber->pointcloud() );

        printf("frame %4d\n", frame);
        fflush(stdout);
        rate.sleep();

        frame++;
    }

    return 0;
}

