#ifndef POINTCLOUD_HUMAN_PREDICTOR_H
#define POINTCLOUD_HUMAN_PREDICTOR_H


/*
 * point cloud data to human shape recognition
 * based on the paper: Real-Time Human Pose Tracking From Range Data
 * simplified model
 */


#include <pcpred/pointcloud/pointcloud.h>
#include <pcpred/shape/human.h>

#include <Eigen/Dense>

#include <vector>


namespace pcpred
{

class PointcloudHumanPredictor
{
public:

    inline void setMaximumIterations(int iteration) { max_iteration_ = iteration; }
    inline void setGradientDescentMaximumiterations(int iteration) { gradient_descent_max_iteration_ = iteration; }

    void observe(const Pointcloud& pointcloud); // redirects to function using a list of centers
    void observe(const std::vector<Eigen::Vector3d>& pointcloud);

private:

    void optimizeHumanShape(const std::vector<Eigen::Vector3d>& pointcloud);

    Human human_shape_;

    int max_iteration_;
    int gradient_descent_max_iteration_;
};

}


#endif // POINTCLOUD_HUMAN_PREDICTOR_H
