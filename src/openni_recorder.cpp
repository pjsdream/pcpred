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
    if (argc != 3 && argc != 4)
    {
        ROS_FATAL("Usage: rosrun pcpred openni_recorder [SEQUENCE_NUMBER] [DURATION(in seconds)] [RATE(optional, in Hz)]");
        return 0;
    }

    const int sequence_number = atoi(argv[1]);
    const int duration = atoi(argv[2]);
    int param_rate = 33;
    if (argc >= 4)
        param_rate = atoi(argv[3]);

    std::string bin_directory = argv[0];
    if (bin_directory.find_last_of('/') == std::string::npos)
        bin_directory = ".";
    else
        bin_directory = bin_directory.substr(0, bin_directory.find_last_of('/'));

    char directory[128];
    char filename[128];
    sprintf(directory, "%s/../data/J%d", bin_directory.c_str(), sequence_number);

    struct stat sb;
    if (!(stat(directory, &sb) == 0 && S_ISDIR(sb.st_mode)) && mkdir(directory, S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH) != 0)
    {
        ROS_FATAL("Failed to make directory [%s]", directory);
        return 0;
    }

    sprintf(filename, "%s/joints.txt", directory);
    FILE* fp = fopen(filename, "w");
    if (fp == 0)
    {
        ROS_FATAL("Failed to access file [%s]", filename);
        return 0;
    }

    ros::init(argc, argv, "openni_recorder");
    ROS_INFO("openni_recorder");
    ros::NodeHandle nh;
    ros::Rate rate(param_rate);

    tf::TransformListener listener;

    std::vector<std::string> joints;
    joints.push_back("head_1");
    joints.push_back("left_elbow_1");
    joints.push_back("left_foot_1");
    joints.push_back("left_hand_1");
    joints.push_back("left_hip_1");
    joints.push_back("left_knee_1");
    joints.push_back("left_shoulder_1");
    joints.push_back("neck_1");
    joints.push_back("torso_1");
    joints.push_back("right_elbow_1");
    joints.push_back("right_foot_1");
    joints.push_back("right_hand_1");
    joints.push_back("right_hip_1");
    joints.push_back("right_knee_1");
    joints.push_back("right_shoulder_1");

    ROS_INFO("Recording sequence %d, duration %lf, rate %d", sequence_number, duration, param_rate);
    for (int i=5; i>=1; i--)
    {
        ROS_INFO("%d", i);
        sleep(1);
    }
    ROS_INFO("Recording has started");
    const double start_time = ros::Time::now().toSec();

    while (ros::Time::now().toSec() - start_time <= duration)
    {
        for (int i=0; i<joints.size(); i++)
        {
            const std::string joint_name = joints[i];

            tf::StampedTransform transform;
            try
            {
                listener.lookupTransform("/map", joint_name.c_str(),
                                         ros::Time(0), transform);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }

            tf::Vector3 tfx = transform.getOrigin();
            const Eigen::Vector3d x(tfx.x(), tfx.y(), tfx.z());

            const double t = transform.stamp_.toSec() - start_time;

            fprintf(fp, "%s %lf %lf %lf %lf\n", joint_name.c_str(), t, x(0), x(1), x(2));
        }

        rate.sleep();
    }

    ROS_INFO("Recording complete\n");
    fclose(fp);

    return 0;
}
