#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <sys/stat.h>
#include <unistd.h>

#include <vector>

#include <stdio.h>


int main(int argc, char** argv)
{
    if (argc != 2)
    {
        ROS_FATAL("Usage: rosrun pcpred playback [SEQUENCE_NUMBER]");
        return 0;
    }

    const int sequence_number = atoi(argv[1]);

    std::string bin_directory = argv[0];
    if (bin_directory.find_last_of('/') == std::string::npos)
        bin_directory = ".";
    else
        bin_directory = bin_directory.substr(0, bin_directory.find_last_of('/'));

    char directory[128];
    sprintf(directory, "%s/../data/J%d", bin_directory.c_str(), sequence_number);

    struct stat sb;
    if (!(stat(directory, &sb) == 0 && S_ISDIR(sb.st_mode)))
    {
        ROS_FATAL("Failed to access directory [%s]", directory);
        return 0;
    }

    char filename[128];
    sprintf(filename, "%s/joints.txt", directory);
    FILE* fp = fopen(filename, "r");
    if (fp == 0)
    {
        ROS_FATAL("Failed to access file [%s]", filename);
        return 0;
    }

    ros::init(argc, argv, "playback");
    ROS_INFO("playback");
    ros::NodeHandle nh;


    char name[128];
    double t, x, y, z;
    const double start_time = ros::Time::now().toSec();
    while (ros::ok() && fscanf(fp, "%s%lf%lf%lf%lf", name, &t, &x, &y, &z) == 5)
    {
        while (ros::Time::now().toSec() - start_time < t);

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(x, y, z) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
    }

    ROS_INFO("Playback complete\n");
    fclose(fp);

    return 0;
}
