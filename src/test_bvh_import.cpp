#include <ros/ros.h>

#include <stdio.h>
#include <pcpred/pcpred.h>

#include <Eigen/Dense>

#include <string>

using namespace pcpred;


int main(int argc, char** argv)
{
    std::string filename = "../data/bvh/walking.bvh";
    if (argc >= 2)
        filename = argv[1];

    ros::init(argc, argv, "test_bvh_import");

    ROS_INFO("test_bvh_import");

    BvhImporter importer;
    MarkerVisualizer visualizer("bvh_test");

    const double radius = 0.1;

    if (!importer.import(filename.c_str()))
    {
        ROS_ERROR("bvh file does not exist [%s]", filename.c_str());
        return 0;
    }

    // centimeters to meters
    importer.scale(0.01);

    // rotate from Y-up to Z-up
    importer.rotate(M_PI / 2.0, Eigen::Vector3d(1.0, 0.0, 0.0));

    const int num_frames = importer.numFrames();
    const int num_joints = importer.numJoints();
    ros::Rate rate(importer.rate());

    for (int joint_index = 0; joint_index < num_joints; joint_index++)
        visualizer.clearUptoCapacity(importer.jointName(joint_index).c_str());

    while (true)
    {
        for (int frame_index = 0; frame_index < num_frames; frame_index++)
        {
            for (int joint_index = 0; joint_index < num_joints; joint_index++)
            {
                const std::string joint_name = importer.jointName(joint_index);
                const Eigen::Affine3d transformation = importer.jointTransformation(frame_index, joint_index);
                const std::vector<Eigen::Vector3d> children_offsets = importer.childrenOffsets(joint_index);

                visualizer.drawSphere(joint_name.c_str(), 0, transformation.translation(), radius);

                for (int child_index = 0; child_index < children_offsets.size(); child_index++)
                {
                }
            }

            rate.sleep();
        }
    }

    return 0;
}

