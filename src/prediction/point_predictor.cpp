#include <pcpred/prediction/point_predictor.h>

using namespace pcpred;


PointPredictor::PointPredictor()
{
}

void PointPredictor::setTimestep(double timestep)
{
    timestep_ = timestep;
    kalman_filter_.setTimestep(timestep);
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
    static bool first = true;

    if (first)
    {
        first = false;

        Eigen::VectorXd z(6);
        z << p, 0, 0, 0;
        kalman_filter_.setCurrentObservation(z);
    }

    else
    {
        Eigen::VectorXd mu;
        Eigen::MatrixXd sigma;

        kalman_filter_.predict(1);
        kalman_filter_.getPredictionResultAtFrame(0, mu, sigma);

        kalman_filter_.addStateHistory(mu, sigma);

        Eigen::VectorXd z(6);
        z << p, (p - last_observation_) / timestep_;

        kalman_filter_.setCurrentObservation(z);
    }

    last_observation_ = p;
}

void PointPredictor::predict(int frame_count)
{
    kalman_filter_.predict(frame_count);
}

void PointPredictor::getPredictionResult(int frame_number, Eigen::Vector3d& mu, Eigen::Matrix3d& sigma)
{
    Eigen::VectorXd mu6;
    Eigen::MatrixXd sigma6;

    kalman_filter_.getPredictionResultAtFrame(frame_number, mu6, sigma6);

    mu = mu6.block(0, 0, 3, 1);
    sigma = sigma6.block(0, 0, 3, 3);
}
