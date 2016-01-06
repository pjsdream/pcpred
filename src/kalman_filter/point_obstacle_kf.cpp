#include <pcpred/kalman_filter/point_obstacle_kf.h>

#include <iostream>

#include <algorithm>

using namespace pcpred;


PointObstacleKalmanFilter::PointObstacleKalmanFilter()
{
    A_ = Eigen::Matrix<double, 6, 6>::Identity();
    B_ = Eigen::Matrix<double, 6, 3>::Zero();
    C_ = Eigen::Matrix<double, 6, 6>::Identity();
    d_sigma_ = Eigen::Matrix<double, 6, 6>::Zero();

    setTimestep(0.1);
    setSensorDiagonalCovariance(0.01);
}

void PointObstacleKalmanFilter::setTimestep(double timestep)
{
    timestep_ = timestep;

    A_.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity() * timestep;

    B_.block(0, 0, 3, 3) = Eigen::Matrix3d::Zero();
    B_.block(3, 0, 3, 3) = Eigen::Matrix3d::Identity() * timestep;
}

void PointObstacleKalmanFilter::setSensorDiagonalCovariance(double v)
{
    d_sigma_.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * v;
    d_sigma_.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity() * (2 * v / timestep_);
}

void PointObstacleKalmanFilter::clearStateHistory()
{
    x_mu_history_.clear();
    x_sigma_history_.clear();
}

void PointObstacleKalmanFilter::addStateHistory(const Eigen::VectorXd& mu, const Eigen::MatrixXd& sigma)
{
    x_mu_history_.push_back(mu);
    x_sigma_history_.push_back(sigma);
}

void PointObstacleKalmanFilter::setCurrentObservation(const Eigen::VectorXd& z)
{
    z_ = z;
}

void PointObstacleKalmanFilter::setAccelerationInferenceWindowSize(int window_size)
{
    window_size_ = window_size;
}


void PointObstacleKalmanFilter::predict(int frame_count)
{
    x_mu_result_.clear();
    x_sigma_result_.clear();

    computeAccelerationAndVariance();

    // In the first step, if measurement is given then kalmna filtering
    // If not, initialize x0 with sensor input data
    if (!x_mu_history_.empty())
        updateControlAndMeasurement(x_mu_history_.back(), x_sigma_history_.back());
    else
        initializeFirstStateWithObservation();

    // For the remaining steps, measurements are not given
    // and the result only relies on control updates
    for (int i=1; i<frame_count; i++)
        updateControlOnly(x_mu_result_.back(), x_sigma_result_.back());
}

void PointObstacleKalmanFilter::computeAccelerationAndVariance()
{
    e_sigma_ = Eigen::Matrix<double, 6, 6>::Zero();

    const int history_count = x_mu_history_.size();

    if (history_count <= 1)
    {
        u_ = Eigen::Vector3d(0, 0, 0);
    }

    else
    {
        Eigen::Vector3d mu1 = (x_mu_history_[history_count - 1].block(3, 0, 3, 1) - x_mu_history_[history_count - 2].block(3, 0, 3, 1)) / timestep_;
        Eigen::Matrix3d sigma1 = (x_sigma_history_[history_count - 1].block(3, 3, 3, 3) + x_sigma_history_[history_count - 2].block(3, 3, 3, 3)) / (timestep_ * timestep_);

        const int window_size = std::min((int)history_count, window_size_);
        int weight = 1;
        for (int i=1; i<window_size - 1; i++)
        {
            Eigen::Vector3d mu2 = (x_mu_history_[history_count - 1 - i].block(3, 0, 3, 1) - x_mu_history_[history_count - 2 - i].block(3, 0, 3, 1)) / timestep_;
            Eigen::Matrix3d sigma2 = (x_sigma_history_[history_count - 1 - i].block(3, 3, 3, 3) + x_sigma_history_[history_count - 2 - i].block(3, 3, 3, 3)) / (timestep_ * timestep_);

            const double t = 1.0 / (weight + 1);
            sigma1 = (1-t) * sigma1 + t * sigma2 + t * (1-t) * (mu1 - mu2) * (mu1 - mu2).transpose();
            mu1 = (1-t) * mu1 + t * mu2;
        }

        u_ = mu1;
        e_sigma_.block(3, 3, 3, 3) = sigma1;
    }
}

void PointObstacleKalmanFilter::initializeFirstStateWithObservation()
{
    Eigen::VectorXd x_mu(6);
    Eigen::MatrixXd x_sigma(6, 6);

    x_mu = z_;
    x_sigma = d_sigma_;

    x_mu_result_.push_back(x_mu);
    x_sigma_result_.push_back(x_sigma);
}

void PointObstacleKalmanFilter::updateControlAndMeasurement(const Eigen::VectorXd& last_x_mu, const Eigen::MatrixXd& last_x_sigma)
{
    Eigen::VectorXd x_mu(6);
    Eigen::Matrix<double, 6, 6> x_sigma;

    Eigen::Matrix<double, 6, 6> K; // kalman gain

    x_mu = A_ * last_x_mu + B_ * u_;
    x_sigma = A_ * last_x_sigma * A_.transpose() + e_sigma_;
    K = x_sigma * C_.transpose() * (C_ * x_sigma * C_.transpose() + d_sigma_).inverse();
    x_mu += K * (z_ - C_ * x_mu);
    x_sigma = (Eigen::MatrixXd::Identity(6, 6) - K * C_) * x_sigma;

    x_mu_result_.push_back(x_mu);
    x_sigma_result_.push_back(x_sigma);
}

void PointObstacleKalmanFilter::updateControlOnly(const Eigen::VectorXd& last_x_mu, const Eigen::MatrixXd& last_x_sigma)
{
    Eigen::VectorXd x_mu(6);
    Eigen::MatrixXd x_sigma(6, 6);

    x_mu = A_ * last_x_mu + B_ * u_;
    x_sigma = A_ * last_x_sigma * A_.transpose() + e_sigma_;

    x_mu_result_.push_back(x_mu);
    x_sigma_result_.push_back(x_sigma);
}


void PointObstacleKalmanFilter::getPredictionResultAtFrame(int i, Eigen::VectorXd& mu, Eigen::MatrixXd& sigma)
{
    mu = x_mu_result_[i];
    sigma = x_sigma_result_[i];
}

