#include <ros/ros.h>

#include <pcpred/learning/qlearning.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>

#include <cstdlib>
#include <vector>
#include <queue>

using namespace pcpred;


static std::queue<int> requests;
static void callbackPlanningRequest(const std_msgs::Int32::ConstPtr& msg)
{
    requests.push(msg->data);
}


int main(int argc, char** argv)
{
    srand(time(NULL));


    std::string bin_directory = argv[0];
    if (bin_directory.find_last_of('/') == std::string::npos)
        bin_directory = ".";
    else
        bin_directory = bin_directory.substr(0, bin_directory.find_last_of('/'));

    char directory[128];
    sprintf(directory, "%s/../data", bin_directory.c_str());

    struct stat sb;
    if (!(stat(directory, &sb) == 0 && S_ISDIR(sb.st_mode)))
    {
        ROS_FATAL("Failed to access directory [%s]", directory);
        return 0;
    }

    char filename[128];
    sprintf(filename, "%s/planning.txt", directory);
    FILE* fp = fopen(filename, "r");
    if (fp == 0)
    {
        ROS_FATAL("Failed to access file [%s]", filename);
        return 0;
    }
    fclose(fp);


    ros::init(argc, argv, "motion_planner");
    ROS_INFO("motion_planner");
    ros::NodeHandle nh;

    ros::Publisher planning_time_publisher_;
    planning_time_publisher_ = nh.advertise<std_msgs::Float64>("planning_time", 100);
    ros::Duration(1.0).sleep();

    ros::Subscriber planning_request_subscriber_;
    planning_request_subscriber_ = nh.subscribe<std_msgs::Int32>("planning_request", 100, callbackPlanningRequest);

    while (ros::ok())
    {
        ROS_INFO("Waiting for planning request");
        while (ros::ok() && requests.empty()) ros::spinOnce();
        if (!ros::ok()) break;
        const int action = requests.front();
        requests.pop();
        ROS_INFO("Planning requested: %d", action);

        double time = 1.0;
        ros::Duration(1.0).sleep();

        std_msgs::Float64 planning_time;
        planning_time.data = time;
        ROS_INFO("Planning time = %lf", planning_time.data);
        planning_time_publisher_.publish(planning_time);
    }

    return 0;
}
