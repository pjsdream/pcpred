#include <ros/ros.h>

#include <pcpred/import/openni_subscriber.h>
#include <pcpred/visualization/pointcloud_visualizer.h>

#include <stdio.h>

#include <time.h>
#include <stdlib.h>

using namespace pcpred;


int main(int argc, char** argv)
{
    int num_frames = 300;
    int sequence_number = 1;

    if (argc >= 2)
    {
        num_frames = atoi(argv[1]);

        if (argc >= 3)
            sequence_number = atoi(argv[2]);
    }

    ros::init(argc, argv, "test_openni_subscriber");
    ROS_INFO("test_openni_subscriber");
    ros::NodeHandle n;

    ros::Rate rate(30);

    for (int i = 5; i > 0; i--)
    {
        printf("%d\n", i);
        fflush(stdout);
        sleep(1);
    }

    OpenniSubscriber* subscriber = OpenniSubscriber::getInstance();
    //subscriber->setPrintMessages(true);

    PointcloudVisualizer visualizer("openni_test");

    subscriber->record(num_frames, sequence_number);

    return 0;
}

