#include <ros/ros.h>

#include <pcpred/feature/human_motion_feature.h>

#include <tf/transform_listener.h>
#include <sys/stat.h>
#include <unistd.h>

#include <vector>

#include <stdio.h>

using namespace pcpred;


int main(int argc, char** argv)
{
    if (argc != 3)
    {
        ROS_FATAL("Usage: rosrun pcpred openni_recorder [SEQUENCE_NUMBER] [DURATION(in seconds)]");
        return 0;
    }

    const int sequence_number = atoi(argv[1]);
    const int duration = atoi(argv[2]);
    int param_rate = 15;

    std::string bin_directory = argv[0];
    if (bin_directory.find_last_of('/') == std::string::npos)
        bin_directory = ".";
    else
        bin_directory = bin_directory.substr(0, bin_directory.find_last_of('/'));

    char directory[128];
    char feature_filename[128];
    char human_filename[128];
    sprintf(directory, "%s/../data", bin_directory.c_str());

    struct stat sb;
    if (!(stat(directory, &sb) == 0 && S_ISDIR(sb.st_mode)) && mkdir(directory, S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH) != 0)
    {
        ROS_FATAL("Failed to make directory [%s]", directory);
        return 0;
    }

    sprintf(feature_filename, "%s/J%d/joints.txt", directory, sequence_number);
    FILE* fp = fopen(feature_filename, "w");
    if (fp == 0)
    {
        ROS_FATAL("Failed to access file [%s]", feature_filename);
        return 0;
    }
    fclose(fp);

    sprintf(human_filename, "%s/human.txt", directory);
    fp = fopen(human_filename, "r");
    if (fp == 0)
    {
        ROS_FATAL("Failed to access file [%s]", human_filename);
        return 0;
    }
    fclose(fp);

    ros::init(argc, argv, "openni_recorder");
    ROS_INFO("openni_recorder");
    ros::NodeHandle nh;
    ros::Rate rate(param_rate);

    HumanMotionFeature feature;
    feature.setVisualizerTopic("human_motion");
    feature.loadHumanJoints(human_filename);

    ROS_INFO("Recording sequence %d, duration %ds, rate %d", sequence_number, duration, param_rate);
    for (int i=3; i>=1; i--)
    {
        ROS_INFO("%d", i);
        fflush(stdout);
        sleep(1);
    }
    ROS_INFO("Recording has started");
    fflush(stdout);
    const double start_time = ros::Time::now().toSec();

    tf::TransformListener listener;
    while (ros::ok() && ros::Time::now().toSec() - start_time <= duration)
    {
        Eigen::VectorXd col(feature.numJoints() * 3);

        for (int i=0; i<feature.numJoints(); i++)
        {
            tf::StampedTransform transform;
            while (ros::ok())
            {
                try
                {
                    listener.lookupTransform("map", feature.jointName(i).c_str(),
                                             ros::Time(0), transform);
                    break;
                }
                catch (tf::TransformException ex)
                {
                }
            }

            tf::Vector3 tfx = transform.getOrigin();
            const Eigen::Vector3d x(tfx.x(), tfx.y(), tfx.z());

            col.block(3*i, 0, 3, 1) = x;
        }

        feature.addFrame(col);
        feature.visualizeHumanMotion();
        rate.sleep();
    }

    ROS_INFO("Recording complete\n");

    return 0;
}
