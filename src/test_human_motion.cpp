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
    const double duration = 2.0;

    HumanMotionFeature feature;
    feature.setCurveShape(n, duration);

    d.sleep();

    while (ros::ok())
    {
        for (int i=0; i<feature.numJoints(); i++)
        {
            const std::string joint_name = feature.jointName(i);

            tf::StampedTransform transform;
            try
            {
                listener.lookupTransform("/map", joint_name.c_str(),
                                         ros::Time(0), transform);
            }
            catch (tf::TransformException ex)
            {
                //ROS_ERROR("%s", ex.what());
                continue;
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
