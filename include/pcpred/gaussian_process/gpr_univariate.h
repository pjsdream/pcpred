#ifndef GPR_UNIVARIATE_H
#define GPR_UNIVARIATE_H


#include <Eigen/Dense>

#include <string>

#include <ros/ros.h>


namespace pcpred
{

class GprUnivariate
{
public:

    GprUnivariate();

    void setVisualizerTopic(const std::string& topic);

    void setObservation(const Eigen::VectorXd& X, const Eigen::VectorXd& Y);
    void setHyperParameters(double l, double sigma_f, double sigma_n);

    void print();

    void regression(const Eigen::VectorXd& X, Eigen::VectorXd& mean, Eigen::MatrixXd& variance);
    void visualizeRegression(const Eigen::VectorXd& X);

private:

    double l_;
    double sigma_f_;
    double sigma_n_;

    Eigen::VectorXd X_;
    Eigen::VectorXd Y_;

    Eigen::MatrixXd K_;
    Eigen::MatrixXd K_inverse_;

    ros::Publisher publisher_;
};

}


#endif // GPR_UNIVARIATE_H
