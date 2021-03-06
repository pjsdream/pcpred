#ifndef POINTS_PREDICTOR_H
#define POINTS_PREDICTOR_H


#include <pcpred/prediction/point_predictor.h>

#include <Eigen/Dense>


namespace pcpred
{

class PointsPredictor
{
public:

    explicit PointsPredictor(int num_spheres);

    inline int numSpheres() { return num_spheres_; }

    void setObservationTimestep(double timestep);
    void setSensorDiagonalCovariance(double v);
    void setAccelerationInferenceWindowSize(int window_size);

    void observe(const std::vector<Eigen::Vector3d>& p);

    void predict(double time_difference, std::vector<Eigen::Vector3d>& mu, std::vector<Eigen::Matrix3d>& sigma);
    void predict(double time_difference, int sphere_index, Eigen::Vector3d& mu, Eigen::Matrix3d& sigma);

private:

    const int num_spheres_;
    std::vector<PointPredictor> point_predictors_;
};

}


#endif // POINTS_PREDICTOR_H
