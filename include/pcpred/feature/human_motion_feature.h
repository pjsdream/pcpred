#ifndef HUMAN_MOTION_FEATURE_H
#define HUMAN_MOTION_FEATURE_H


#include <Eigen/Dense>

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

    HumanMotionFeature();

    void loadHumanJoints(const std::string& filename);
    void clear();
    void clearFeature();
    void loadFeature(const std::string& filename);
    void saveFeature(const std::string& filename);

    void addFrame(const Eigen::VectorXd& column);
    void retainLastFrames(int count);

    inline int numFrames()
    {
        return feature_.cols();
    }

    inline int numJoints()
    {
        return joint_names_.size();
    }

    inline std::string jointName(int i)
    {
        return joint_names_[i];
    }

    Eigen::MatrixXd feature();
    Eigen::VectorXd columnFeature();
    int columnFeatureSize();
    int frameSize();

    void setVisualizerTopic(const std::string& topic);
    void visualizeHumanMotion(const std::string& ns = "human");
    void visualizeHumanMotion(int start, int end, const std::string& ns = "human");

private:

    std::vector<std::string> joint_names_;
    std::map<std::string, int> map_joint_to_index_;
    std::vector<std::pair<int, int> > links_;

    Eigen::MatrixXd feature_;

    ros::Publisher publisher_;
};

}


#endif // HUMAN_MOTION_FEATURE_H
