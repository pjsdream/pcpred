#include <ros/ros.h>

#include <pcpred/learning/learning_motion.h>

#include <tf/transform_broadcaster.h>

#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>

#include <cstdlib>
#include <vector>

using namespace pcpred;


int main(int argc, char** argv)
{
    if (argc != 2)
    {
        ROS_FATAL("Usage: rosrun pcpred learn_motion [SEQUENCE_NUMBER]");
        return 0;
    }

    srand(time(NULL));

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
    fclose(fp);

    char params_filename[128];
    sprintf(params_filename, "%s/learning_options.txt", directory);
    FILE* pfp = fopen(filename, "r");
    if (pfp == 0)
    {
        ROS_FATAL("Failed to access file [%s]", filename);
        return 0;
    }
    fclose(pfp);

    ros::init(argc, argv, "learn_motion");
    ROS_INFO("learn_motion");
    ros::NodeHandle nh;

    double start_time;


    LearningMotionOptions learning_options;
    learning_options.parse(params_filename);
    learning_options.print();
    fflush(stdout);

    LearningMotion learning_motion;
    learning_motion.setVerbose();
    learning_motion.setOptions(learning_options);
    learning_motion.parseData(filename);

    ROS_INFO("Learning start");
    fflush(stdout);
    start_time = ros::Time::now().toSec();
    learning_motion.learn();
    ROS_INFO("Laerning complete in %lf sec\n", ros::Time::now().toSec() - start_time);
    fflush(stdout);


    return 0;
}
