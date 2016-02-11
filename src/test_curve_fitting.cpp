#include <ros/ros.h>

#include <pcpred/feature/stream_curve_fitting.h>

#include <tf/transform_listener.h>

#include <vector>

#include <stdio.h>

using namespace pcpred;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "curve_fitting");
    ROS_INFO("curve_fitting");
    ros::NodeHandle nh;
    ros::Duration d(1.0);

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

    std::vector<StreamCurveFitting> curves(joints.size());
    for (int i=0; i<curves.size(); i++)
        curves[i].setCurveShape(n, duration);

    d.sleep();

    while (true)
    {
        for (int i=0; i<curves.size(); i++)
        {
            tf::StampedTransform transform;
            try
            {
                listener.lookupTransform("/map", joints[i].c_str(),
                                         ros::Time(0), transform);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }

            tf::Vector3 tfx = transform.getOrigin();
            Eigen::Vector3d x(tfx.x(), tfx.y(), tfx.z());

            const double t = transform.stamp_.toSec();
            curves[i].push(t, x);
            curves[i].fit();
            curves[i].visualizeCurve(i);
        }
    }

    d.sleep();

    return 0;
}
