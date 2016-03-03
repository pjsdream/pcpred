#ifndef IVM_H
#define IVM_H


#include <Eigen/Dense>

#include <vector>


namespace pcpred
{

class Ivm
{
public:

    Ivm();

    void setKernelCoefficients(double l, double sigma_f, double sigma_n);
    void setLambda(double lambda);

    void train(const Eigen::MatrixXd& X, const Eigen::VectorXi& c);

    Eigen::VectorXd getProbability(const Eigen::VectorXd& x);

private:

    double kernel(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2);

    Eigen::MatrixXd X_;
    Eigen::VectorXi c_;
    int C_;

    double lambda_;
    double l_;
    double sigma_f_;
    double sigma_n_;

    std::vector<int> s_indices_;
};

}


#endif // IVM_H
