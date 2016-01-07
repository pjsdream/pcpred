#ifndef VISUALIZER_H
#define VISUALIZER_H


#include <Eigen/Dense>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include <map>
#include <string>


namespace pcpred
{

class Visualizer
{
private:
    static const int CAPACITY = 1000;

    static std::map<double, double> gaussian_distribution_radius_table_;
    static bool gaussian_distribution_radius_table_initialized_;

public:

    explicit Visualizer(const char* topic = "visualization_marker");

    void clear(const char* ns);
    void clearUptoCapacity(const char* ns);

    void drawSphere(const char* ns, const Eigen::Vector3d& center, double radius);
    void drawGaussianDistribution(const char* ns, const Eigen::Vector3d& mu, const Eigen::Matrix3d& sigma, double probability, double offset = 0.0);

private:

    void publish(const visualization_msgs::Marker& marker);

    ros::Publisher publisher_;

    std::map<std::string, int> count_;
};

}


#endif // VISUALIZER_H
