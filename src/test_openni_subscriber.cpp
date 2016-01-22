#include <ros/ros.h>

#include <pcpred/import/openni_subscriber.h>
#include <pcpred/visualization/pointcloud_visualizer.h>

#include <stdio.h>

#include <time.h>
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

    for (int i = 5; i > 0; i--)
    {
        printf("%d\n", i);
        fflush(stdout);
        sleep(1);
    }
    subscriber->record(300, 1);

    return 0;
}

