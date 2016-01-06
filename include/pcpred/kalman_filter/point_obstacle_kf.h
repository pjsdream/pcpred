#ifndef POINT_OBSTACLE_KF_H
#define POINT_OBSTACLE_KF_H


#include <Eigen/Dense>

#include <vector>


namespace pcpred
{

class PointObstacleKalmanFilter
{
public:

    PointObstacleKalmanFilter();

    void setTimestep(double timestep);
    void setSensorDiagonalCovariance(double v);
    void clearStateHistory();
    void addStateHistory(const Eigen::VectorXd& mu, const Eigen::MatrixXd& sigma);
    void setCurrentObservation(const Eigen::VectorXd& z);
    void setAccelerationInferenceWindowSize(int window_size);

    void predict(int frame_count);

    void getPredictionResultAtFrame(int i, Eigen::VectorXd& mu, Eigen::MatrixXd& sigma);

private:

    // These functions receive the gaussian distribution parameters of last state
    // The result is pushed back to result arrays
    void initializeFirstStateWithObservation();
    void updateControlAndMeasurement(const Eigen::VectorXd& last_x_mu, const Eigen::MatrixXd& last_x_sigma);
    void updateControlOnly(const Eigen::VectorXd& last_x_mu, const Eigen::MatrixXd& last_x_sigma);

    // acceleratoin inference
    void computeAccelerationAndVariance();

    // timestep for robot control variab
    double timestep_;

    // acceleration inference window size
    int window_size_;

    // kalman filter variables
    // x_t = A x_{t-1} + B u_t + e_t
    // z_t = C x_t + d_t
    Eigen::Matrix<double, 6, 6> A_;
    Eigen::Matrix<double, 6, 3> B_;
    Eigen::Matrix<double, 6, 6> C_;
    std::vector<Eigen::VectorXd> x_mu_history_;
    std::vector<Eigen::MatrixXd> x_sigma_history_;
    Eigen::Vector3d u_;
    Eigen::Matrix<double, 6, 6> e_sigma_;
    Eigen::VectorXd z_;

    // u_t is acceleration vector, determined by history of states
    // e_t ~ N(0, sigma_e_t) where sigma is determined by history
    // d_t ~ N(0, sigma_d_t)
    Eigen::Matrix<double, 6, 6> d_sigma_;

    // kalman filter result
    std::vector<Eigen::VectorXd> x_mu_result_;
    std::vector<Eigen::MatrixXd> x_sigma_result_;
};

}


#endif // POINT_OBSTACLE_KF_H
