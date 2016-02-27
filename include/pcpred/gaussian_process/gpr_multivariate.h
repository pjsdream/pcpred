#ifndef GPR_MULTIVARIATES_H
#define GPR_MULTIVARIATES_H


#include <Eigen/Dense>

#include <string>

#include <ros/ros.h>


namespace pcpred
{

class GprMultivariate
{
public:

    GprMultivariate();

    void setVisualizerTopic(const std::string& topic);

    void setObservation(const Eigen::MatrixXd &X, const Eigen::MatrixXd& Y);
    void setObservationInput(const Eigen::MatrixXd& X);
    void setObservationOutput(const Eigen::VectorXd &Y);
    void setHyperParameters(double l, double sigma_f, double sigma_n);

    void print();

    void regression(const Eigen::MatrixXd& X, Eigen::VectorXd& mean, Eigen::MatrixXd& variance);
    void regression(const Eigen::VectorXd& x, double& mean, double& variance);
    void visualizeInput();
    void visualizeRegression(const Eigen::MatrixXd& X);

private:

    double kernel(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2);

    // output dimension
    int D_;

    double l_;
    double sigma_f_;
    double sigma_n_;

    Eigen::MatrixXd X_;
    Eigen::MatrixXd Y_;

    Eigen::MatrixXd K_;
    Eigen::MatrixXd K_inverse_;

    ros::Publisher publisher_;
};

}


#endif // GPR_MULTIVARIATES_H
