#include <ros/ros.h>

#include <stdio.h>
#include <pcpred/pcpred.h>

#include <string>

using namespace pcpred;


int main(int argc, char** argv)
{
    std::string filename = "../data/bvh/walking_inPlace.bvh";
    if (argc >= 2)
        filename = argv[1];

    ros::init(argc, argv, "test_bvh_import");

    ROS_INFO("test_bvh_import");

    BvhImporter importer;
    MarkerVisualizer visualizer("bvh_test");

    ros::Rate rate(33);

    importer.import(filename.c_str());

    const int num_frames = importer.numFrames();
    for (int frame_index = 0; frame_index < num_frames; frame_idnex++)
    {
    }

    return 0;
}

