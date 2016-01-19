#ifndef POINTCLOUD_HUMAN_DETECTOR_H
#define POINTCLOUD_HUMAN_DETECTOR_H


#include <pcpred/visualization/marker_array_visualizer.h>
#include <pcpred/pointcloud/pointcloud.h>

#include <Eigen/Dense>

#include <vector>


namespace pcpred
{

class PointcloudHumanDetector
{
public:

    PointcloudHumanDetector();

    inline void setSphereRadius(double radius) { sphere_radius_ = radius; }
    inline void setNumberSpheres(int n) { num_spheres_ = n; }

    void observe(const Pointcloud& pointcloud);

    void setVisualizerTopic(const char* topic);
    void visualizeHuman();

private:

    MarkerArrayVisualizer* visualizer_;

    int num_spheres_;
    double sphere_radius_;
    std::vector<Eigen::Vector3d> sphere_centers_;
};

}


#endif // POINTCLOUD_HUMAN_DETECTOR_H
