#ifndef HUMAN_MOTION_FEATURE_H
#define HUMAN_MOTION_FEATURE_H


#include <Eigen/Dense>

#include <pcpred/feature/hermite_curve.h>
#include <pcpred/feature/stream_curve_fitting.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>
#include <map>


namespace pcpred
{

class HumanMotionFeature
{
public:
    enum FeatureType
    {
        FEATURE_TYPE_ABSOLUTE_POSITION = 0,
    };

public:

    HumanMotionFeature();

    void setJointNames(const std::vector<std::string>& joints);
    void setCurveShape(int num_pieces, double duration);
    void addLink(const std::string& joint1, const std::string& joint2);

    void observe(const std::string& joint_name, double time, const Eigen::Vector3d& joint_position);

    Eigen::VectorXd toFeature(FeatureType feature_type);

    void setVisualizerTopic(const std::string& topic);
    void visualizeHumanMotion();

private:

    Eigen::VectorXd toFeatureAbsolutePosition();

    void appendTraceMarkers(double size, const std_msgs::ColorRGBA& color0, const std_msgs::ColorRGBA& color1, visualization_msgs::MarkerArray& marker_array);
    void appendJointMarkers(double time, double size, const std_msgs::ColorRGBA& color, visualization_msgs::MarkerArray& marker_array);
    void appendSkeletonMarkers(double time, double size, const std_msgs::ColorRGBA& color, visualization_msgs::MarkerArray& marker_array);

    std::map<std::string, int> map_joint_to_index_;
    std::vector<std::pair<int, int> > links_;

    int num_pieces_;
    double duration_;

    std::vector<StreamCurveFitting> streams_;
    std::vector<HermiteCurve> curves_;

    ros::Publisher publisher_;
};

}


#endif // HUMAN_MOTION_FEATURE_H
