#include <pcpred/learning/kmeans.h>

using namespace pcpred;


Kmeans::Kmeans()
{
    setTerminationCondition(1000);
}

void Kmeans::setTerminationCondition(int max_iterations)
{
    max_iterations_ = max_iterations;
}

std::vector<int> Kmeans::cluster(const Eigen::MatrixXd& X, int k)
{
    const int n = X.cols();
    const int d = X.rows();

    Eigen::MatrixXd c(d, k);
    std::vector<char> selected(n, 0);
    std::vector<int> num(k, 0);

    result_.resize(n, 0);

    // initial centers
    for (int i=0; i<k; i++)
    {
        int x;
        do
        {
            x = rand() % n;
        } while(!selected[x]);

        selected[x] = true;
        c.col(i) = X.col(x);
    }

    for (int iteration = 0; iteration < max_iterations_; iteration++)
    {
        bool changed = false;

        // assignment
        for (int i=0; i<n; i++)
        {
            double minimum_squared_norm = (X.col(i) - c.col(result_[i])).squaredNorm();
            for (int j=0; j<k; j++)
            {
                const double squared_norm = (X.col(i) - c.col(j)).squaredNorm();

                if (minimum_squared_norm > squared_norm)
                {
                    changed = true;
                    minimum_squared_norm = squared_norm;
                    result_[i] = j;
                }
            }
        }

        // moving center
        c.setZero();
        for (int i=0; i<k; i++)
            num[i] = 0;

        for (int i=0; i<n; i++)
        {
            c.col(result_[i]) += X.col(i);
            num[result_[i]]++;
        }

        for (int i=0; i<k; i++)
            c.col(i) /= result_[i];

        // terminate if converged
        if (!changed)
            break;
    }

    return result_;
}

std::vector<int> Kmeans::cluster(const Eigen::MatrixXd& X, const std::vector<int>& indices, int k)
{
    const int n = indices.size();
    const int d = X.rows();

    Eigen::MatrixXd c(d, k);
    std::vector<char> selected(n, 0);
    std::vector<int> num(k, 0);

    result_.resize(n, 0);

    // initial centers
    for (int i=0; i<k; i++)
    {
        int x;
        do
        {
            x = rand() % n;
        } while(selected[x]);

        selected[x] = true;
        c.col(i) = X.col(x);
    }

    for (int iteration = 0; iteration < max_iterations_; iteration++)
    {
        bool changed = false;

        // assignment
        for (int i=0; i<n; i++)
        {
            double minimum_squared_norm = (X.col(indices[i]) - c.col(result_[i])).squaredNorm();
            for (int j=0; j<k; j++)
            {
                const double squared_norm = (X.col(indices[i]) - c.col(j)).squaredNorm();

                if (minimum_squared_norm > squared_norm)
                {
                    changed = true;
                    minimum_squared_norm = squared_norm;
                    result_[i] = j;
                }
            }
        }

        // moving center
        c.setZero();
        for (int i=0; i<k; i++)
            num[i] = 0;

        for (int i=0; i<n; i++)
        {
            c.col(result_[i]) += X.col(indices[i]);
            num[result_[i]]++;
        }

        bool empty_cluster = false;
        for (int i=0; i<k; i++)
        {
            if (num[i] == 0)
            {
                empty_cluster = true;
                break;
            }
            else
                c.col(i) /= num[i];
        }

        if (empty_cluster)
        {
            selected.resize(n, 0);
            for (int i=0; i<k; i++)
            {
                int x;
                do
                {
                    x = rand() % n;
                } while(selected[x]);

                selected[x] = true;
                c.col(i) = X.col(x);
            }
        }

        // terminate if converged
        if (!changed)
            break;
    }

    return result_;
}
