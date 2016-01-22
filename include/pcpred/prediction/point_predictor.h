#ifndef POINT_PREDICTOR_H
#define POINT_PREDICTOR_H


#include <pcpred/kalman_filter/point_obstacle_kf.h>

#include <Eigen/Dense>


namespace pcpred
{

class PointPredictor
{
public:

    PointPredictor();

    void setObservationTimestep(double timestep);
    void setSensorDiagonalCovariance(double v);
    void setAccelerationInferenceWindowSize(int window_size);

    void observe(const Eigen::Vector3d& p);

    void predict(double time_difference, Eigen::Vector3d& mu, Eigen::Matrix3d& sigma);

private:

    bool first_;

    PointObstacleKalmanFilter kalman_filter_;
    double timestep_;
    Eigen::VectorXd last_observation_;
};

}


#endif // POINT_PREDICTOR_H
