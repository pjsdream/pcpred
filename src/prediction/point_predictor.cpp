#include <pcpred/prediction/point_predictor.h>

using namespace pcpred;


PointPredictor::PointPredictor()
{
    first_ = true;
}

void PointPredictor::setObservationTimestep(double timestep)
{
    timestep_ = timestep;
    kalman_filter_.setObservationTimestep(timestep);
}

void PointPredictor::setSensorDiagonalCovariance(double v)
{
    kalman_filter_.setSensorDiagonalCovariance(v);
}

void PointPredictor::setAccelerationInferenceWindowSize(int window_size)
{
    kalman_filter_.setAccelerationInferenceWindowSize(window_size);
}


void PointPredictor::observe(const Eigen::Vector3d& p)
{
    if (first_)
    {
        first_ = false;

        Eigen::VectorXd z(6);
        z << p, 0, 0, 0;
        kalman_filter_.setCurrentObservation(z);
    }

    else
    {
        Eigen::VectorXd z(6);
        z << p, (p - last_observation_) / timestep_;

        kalman_filter_.setCurrentObservation(z);
    }

    last_observation_ = p;
}

void PointPredictor::predict(double time_difference, Eigen::Vector3d& mu, Eigen::Matrix3d& sigma)
{
    Eigen::VectorXd mu6;
    Eigen::MatrixXd sigma6;

    kalman_filter_.predict(time_difference, mu6, sigma6);

    mu = mu6.block(0, 0, 3, 1);
    sigma = sigma6.block(0, 0, 3, 3);
}
