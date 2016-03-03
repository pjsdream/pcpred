#include <pcpred/learning/ivm.h>

using namespace pcpred;


Ivm::Ivm()
{
}

double Ivm::kernel(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2)
{
    return sigma_f_ * sigma_f_ * std::exp( -(x1-x2).squaredNorm() / (2. * l_ * l_));
}

void Ivm::train(const Eigen::MatrixXd &X, const Eigen::VectorXi &c)
{
    X_ = X;
    c_ = c;
    C_ = c.maxCoeff();

    const int n = X.cols();

    s_indices_.clear();
    std::vector<char> selected(n, false);

    Eigen::MatrixXd K(n, n);
    for (int i=0; i<n; i++)
    {
        for (int j=0; j<n; j++)
        {
            K(i,j) = kernel(X.col(i), X.col(j));

            if (i==j)
                K(i,j) += sigma_n_ * sigma_n_;
        }
    }

    for (int i=0; i<n; i++)
    {
        if (!selected[i])
        {
        }
    }
}

Eigen::VectorXd Ivm::getProbability(const Eigen::VectorXd &x)
{
    Eigen::VectorXd p;

    return p;
}
