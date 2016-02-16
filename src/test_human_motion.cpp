#include <ros/ros.h>

#include <pcpred/feature/human_motion_feature.h>

#include <tf/transform_listener.h>

#include <vector>

#include <stdio.h>

using namespace pcpred;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "human_motion");
    ROS_INFO("human_motion");
    ros::NodeHandle nh;
    ros::Duration d(1.0);
    ros::Rate rate(33);

    tf::TransformListener listener;

    const int n = 5;
    const double duration = 1.0;

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

    HumanMotionFeature feature;
    feature.setCurveShape(n, duration);
    feature.setJointNames(joints);

    feature.addLink("head_1", "neck_1");
    feature.addLink("neck_1", "left_shoulder_1");
    feature.addLink("left_shoulder_1", "left_elbow_1");
    feature.addLink("left_elbow_1", "left_hand_1");
    feature.addLink("neck_1", "right_shoulder_1");
    feature.addLink("right_shoulder_1", "right_elbow_1");
    feature.addLink("right_elbow_1", "right_hand_1");
    feature.addLink("neck_1", "torso_1");
    feature.addLink("torso_1", "left_hip_1");
    feature.addLink("left_hip_1", "left_knee_1");
    feature.addLink("left_knee_1", "left_foot_1");
    feature.addLink("torso_1", "right_hip_1");
    feature.addLink("right_hip_1", "right_knee_1");
    feature.addLink("right_knee_1", "right_foot_1");

    d.sleep();

    while (true)
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

            const double t = transform.stamp_.toSec();
            feature.observe(joint_name, t, x);
        }

        feature.visualizeHumanMotion();

        rate.sleep();
    }

    return 0;
}
