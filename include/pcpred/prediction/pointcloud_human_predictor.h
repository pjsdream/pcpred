#ifndef POINTCLOUD_HUMAN_PREDICTOR_H
#define POINTCLOUD_HUMAN_PREDICTOR_H


/*
 * point cloud data to human shape recognition
 * based on the paper: Real-Time Human Pose Tracking From Range Data
 * simplified model
 */


#include <pcpred/pointcloud/pointcloud.h>
#include <pcpred/shape/human.h>
#include <pcpred/prediction/points_predictor.h>

#include <Eigen/Dense>

#include <vector>


namespace pcpred
{

class PointcloudHumanPredictor
{
public:

    PointcloudHumanPredictor();

    // optimization parameters
    inline void setMaximumIterations(int iteration) { max_iteration_ = iteration; }
    inline void setGradientDescentMaximumIterations(int iteration) { gradient_descent_max_iteration_ = iteration; }
    inline void setGradientDescentAlpha(double alpha) { gradient_descent_alpha_ = alpha; }

    // capsule human model to spheres
    inline void setCapsuleDivisor(int d) { capsule_divisor_ = d; }

    int numSpheres();

    void loadHumanShapeFromFile(const char* filename);

    void observe(const Pointcloud& pointcloud); // redirects to function using a list of centers
    void observe(const std::vector<Eigen::Vector3d>& pointcloud);

    void predict(int frame_count);

    void getPredictionResult(int frame_number, int sphere_index, Eigen::Vector3d& mu, Eigen::Matrix3d& sigma);
    void getPredictionResults(int frame_number, std::vector<Eigen::Vector3d>& mu, std::vector<Eigen::Matrix3d>& sigma);

private:

    void optimizeHumanShape(const std::vector<Eigen::Vector3d>& pointcloud);

    Human human_shape_;

    PointsPredictor* predictor_;

    int max_iteration_;
    int gradient_descent_max_iteration_;
    double gradient_descent_alpha_;

    int capsule_divisor_;
};

}


#endif // POINTCLOUD_HUMAN_PREDICTOR_H