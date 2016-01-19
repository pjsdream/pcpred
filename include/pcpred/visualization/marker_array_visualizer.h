#ifndef MARKER_ARRAY_VISUALIZER_H
#define MARKER_ARRAY_VISUALIZER_H


#include <pcpred/visualization/visualizer.h>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include <map>
#include <string>


namespace pcpred
{

class MarkerArrayVisualizer : public Visualizer
{
private:

    static const int CAPACITY = 1000;

public:

    explicit MarkerArrayVisualizer(const char* topic = "visualization_marker_array");

    void drawSphere(const char* ns, const Eigen::Vector3d& center, double radius);
    void drawSpheres(const char* ns, const std::vector<Eigen::Vector3d>& center, const std::vector<double> radius);
    void drawEllipsoids(const char* ns, const std::vector<Eigen::Vector3d>& center, const std::vector<Eigen::Matrix3d>& A);
    void drawSphereList(const char* ns, const std::vector<Eigen::Vector3d>& centers, double radius);
    void drawLineStrip(const char* ns, const std::vector<Eigen::Vector3d>& endpoints);
    void drawLineList(const char* ns, const std::vector<Eigen::Vector3d>& endpoints);

private:

    void publish(const visualization_msgs::MarkerArray& marker_array);
};

}


#endif // MARKER_ARRAY_VISUALIZER_H
