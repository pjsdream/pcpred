#ifndef POINT_OBSTACLE_KF_H
#define POINT_OBSTACLE_KF_H


#include <Eigen/Dense>


namespace pcpred
{

class GaussianDistribution
{
public:

    GaussianDistribution(int dimension);

private:

    const int dimension_;

    Eigen::VectorXd mu_;
    Eigen::MatrixXd sigma_;
};


class PointObstacleKalmanFilter
{
public:

private:

    // timestep
    double timestep_;

    // kalman filter matrices
    // x_t = A x_{t-1} + B u_t + e_t
    // z_t = C x_t + d_t
    Eigen::Matrix4d A_;
    Eigen::MatrixXd B_;
    Eigen::MatrixXd C_;
};

}


#endif // POINT_OBSTACLE_KF_H
