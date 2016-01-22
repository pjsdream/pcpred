#include <pcpred/kalman_filter/point_obstacle_kf.h>

#include <iostream>

#include <algorithm>

#include <stdio.h>


using namespace pcpred;


PointObstacleKalmanFilter::PointObstacleKalmanFilter()
{
    A_ = Eigen::Matrix<double, 6, 6>::Identity();
    B_ = Eigen::Matrix<double, 6, 3>::Zero();
    B_.block(3, 0, 3, 3) = Eigen::Matrix3d::Identity();
    C_ = Eigen::Matrix<double, 6, 6>::Identity();
    d_sigma_ = Eigen::Matrix<double, 6, 6>::Zero();

    setObservationTimestep(0.1);
    setSensorDiagonalCovariance(0.01);
    setAccelerationInferenceWindowSize(2);
}

void PointObstacleKalmanFilter::setObservationTimestep(double timestep)
{
    timestep_ = timestep;

    A_.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity() * timestep;
}

void PointObstacleKalmanFilter::setSensorDiagonalCovariance(double v)
{
    d_sigma_.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * v;
    d_sigma_.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity() * (2 * v / (timestep_ * timestep_));
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

    computeAccelerationAndVariance();

    Eigen::VectorXd mu;
    Eigen::MatrixXd sigma;

    // In the first step, if measurement is given then kalmna filtering
    // If not, initialize x0 with sensor input data
    if (!x_mu_history_.empty())
        updateControlAndMeasurement(x_mu_history_.back(), x_sigma_history_.back(), mu, sigma);
    else
        initializeFirstStateWithObservation(mu, sigma);

    addStateHistory(mu, sigma);
}

void PointObstacleKalmanFilter::setAccelerationInferenceWindowSize(int window_size)
{
    window_size_ = window_size;
}


void PointObstacleKalmanFilter::predict(double time_difference, Eigen::VectorXd& mu, Eigen::MatrixXd& sigma)
{
    updateControlOnly(time_difference, x_mu_history_.back(), x_sigma_history_.back(), mu, sigma);
}

void PointObstacleKalmanFilter::computeAccelerationAndVariance()
{
    const double motion_variance = 0.01 * timestep_ * timestep_;

    e_sigma_ = Eigen::Matrix<double, 6, 6>::Zero();
    e_sigma_.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity() * motion_variance;
    u_ = Eigen::Vector3d(0, 0, 0);

    const int history_count = x_mu_history_.size();

    if (history_count <= 1)
    {
        u_ = Eigen::Vector3d(0, 0, 0);
    }

    else
    {
        Eigen::Vector3d mu1 = x_mu_history_[history_count - 1].block(3, 0, 3, 1) - x_mu_history_[history_count - 2].block(3, 0, 3, 1);
        Eigen::Matrix3d sigma1 = x_sigma_history_[history_count - 1].block(3, 3, 3, 3) + x_sigma_history_[history_count - 2].block(3, 3, 3, 3);

        const int window_size = std::min((int)history_count, window_size_);
        int weight = 1;
        for (int i=1; i<window_size - 1; i++)
        {
            Eigen::Vector3d mu2 = x_mu_history_[history_count - 1 - i].block(3, 0, 3, 1) - x_mu_history_[history_count - 2 - i].block(3, 0, 3, 1);
            Eigen::Matrix3d sigma2 = x_sigma_history_[history_count - 1 - i].block(3, 3, 3, 3) + x_sigma_history_[history_count - 2 - i].block(3, 3, 3, 3);

            const double t = 1.0 / (weight + 1);
            sigma1 = (1-t) * sigma1 + t * sigma2 + t * (1-t) * (mu1 - mu2) * (mu1 - mu2).transpose();
            mu1 = (1-t) * mu1 + t * mu2;

            weight++;
        }

        u_ = mu1 * timestep_;
        e_sigma_.block(3, 3, 3, 3) += sigma1 * timestep_ * timestep_;
    }
}

void PointObstacleKalmanFilter::initializeFirstStateWithObservation(Eigen::VectorXd& mu, Eigen::MatrixXd& sigma)
{
    mu = z_;
    sigma = d_sigma_;
}

void PointObstacleKalmanFilter::updateControlAndMeasurement(const Eigen::VectorXd& last_x_mu, const Eigen::MatrixXd& last_x_sigma, Eigen::VectorXd& mu, Eigen::MatrixXd& sigma)
{
    Eigen::Matrix<double, 6, 6> K; // kalman gain

    mu = A_ * last_x_mu + B_ * u_;
    sigma = A_ * last_x_sigma * A_.transpose() + e_sigma_;
    K = sigma * C_.transpose() * (C_ * sigma * C_.transpose() + d_sigma_).inverse();
    mu += K * (z_ - C_ * mu);
    sigma = (Eigen::MatrixXd::Identity(6, 6) - K * C_) * sigma;
}

void PointObstacleKalmanFilter::updateControlOnly(double time_difference, const Eigen::VectorXd& last_x_mu, const Eigen::MatrixXd& last_x_sigma, Eigen::VectorXd& mu, Eigen::MatrixXd& sigma)
{
    Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Identity();
    A.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity() * time_difference;

    mu = A * last_x_mu + B_ * (u_ / timestep_ * time_difference);
    sigma = A * last_x_sigma * A.transpose() + (e_sigma_ / timestep_ / timestep_ * time_difference * time_difference);
}
