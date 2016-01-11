#ifndef MARKER_VISUALIZER_H
#define MARKER_VISUALIZER_H


#include <pcpred/visualization/visualizer.h>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <map>
#include <string>


namespace pcpred
{

class MarkerVisualizer : public Visualizer
{
private:
    static const int CAPACITY = 1000;

    static std::map<double, double> gaussian_distribution_radius_table_;
    static bool gaussian_distribution_radius_table_initialized_;

public:

    explicit MarkerVisualizer(const char* topic = "visualization_marker");

    void clear(const char* ns, int id);
    void clearUptoCapacity(const char* ns);

    void drawSphere(const char* ns, int id, const Eigen::Vector3d& center, double radius);
    void drawGaussianDistribution(const char* ns, int id, const Eigen::Vector3d& mu, const Eigen::Matrix3d& sigma, double probability, double offset = 0.0);

private:

    void publish(const visualization_msgs::Marker& marker);
};

}


#endif // MARKER_VISUALIZER_H
