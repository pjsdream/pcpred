#ifndef HIERARCHICAL_KMEANS_H
#define HIERARCHICAL_KMEANS_H


#include <vector>

#include <Eigen/Dense>

#include <pcpred/learning/kmeans.h>


namespace pcpred
{

class HierarchicalKmeans
{
public:

    HierarchicalKmeans();

    void setK(int k);
    void setSizeLimit(int size_limit);
    void setTerminationCondition(int max_iterations);

    void setVerbose(bool flag = true);

    std::vector<int> clusterSizeConstraint(const Eigen::MatrixXd& X);
    std::vector<int> clusterSizeConstraint(const Eigen::MatrixXd& X, const std::vector<int>& indices);

private:

    int k_;
    int size_limit_;
    Kmeans kmeans_;

    bool verbose_;
};

}


#endif // HIERARCHICAL_KMEANS_H
