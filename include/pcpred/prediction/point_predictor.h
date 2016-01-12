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

    void setTimestep(double timestep);
    void setSensorDiagonalCovariance(double v);
    void setAccelerationInferenceWindowSize(int window_size);

    void observe(const Eigen::Vector3d& p);

    void predict(int frame_count);

    void getPredictionResult(int frame_number, Eigen::Vector3d& mu, Eigen::Matrix3d& sigma);

private:

    bool first_;

    PointObstacleKalmanFilter kalman_filter_;
    double timestep_;
    Eigen::VectorXd last_observation_;
};

}


#endif // POINT_PREDICTOR_H
