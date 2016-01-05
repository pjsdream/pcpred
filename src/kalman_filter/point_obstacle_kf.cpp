#include <pcpred/kalman_filter/point_obstacle_kf.h>

using namespace pcpred;


GaussianDistribution::GaussianDistribution(int dimension)
    : dimension_(dimension)
{
    mu_ = Eigen::Vector3d(dimension);
    sigma_ = Eigen::MatrixXd(dimension, dimension);
}
