#ifndef KMEANS_H
#define KMEANS_H


#include <Eigen/Dense>

#include <vector>


namespace pcpred
{

class Kmeans
{
public:

    Kmeans();

    void setTerminationCondition(int max_iterations);

    std::vector<int> cluster(const Eigen::MatrixXd& X, int k);
    std::vector<int> cluster(const Eigen::MatrixXd& X, const std::vector<int>& indices, int k);

private:

    int max_iterations_;
    double tolerance_;

    std::vector<int> result_;
};

}


#endif // KMEANS_H
