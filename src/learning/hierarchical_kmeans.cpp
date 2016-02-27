#include <pcpred/learning/hierarchical_kmeans.h>

#include <stdio.h>

#include <map>

using namespace pcpred;


HierarchicalKmeans::HierarchicalKmeans()
{
    kmeans_.setTerminationCondition(1000);

    setK(2);
    setSizeLimit(100);
}

void HierarchicalKmeans::setK(int k)
{
    k_ = k;
}

void HierarchicalKmeans::setSizeLimit(int size_limit)
{
    size_limit_ = size_limit;
}

void HierarchicalKmeans::setTerminationCondition(int max_iterations)
{
    kmeans_.setTerminationCondition(max_iterations);
}

std::vector<int> HierarchicalKmeans::clusterSizeConstraint(const Eigen::MatrixXd &X)
{
    std::vector<int> indices(X.cols());
    for (int i=0; i<indices.size(); i++)
        indices[i] = i;

    return clusterSizeConstraint(X, indices);
}

std::vector<int> HierarchicalKmeans::clusterSizeConstraint(const Eigen::MatrixXd &X, const std::vector<int>& indices)
{
    const int n = indices.size();

    if (n <= size_limit_)
        return std::vector<int>(n, 0);

    std::vector<int> result(n);
    std::vector<int> current_cluster_result = kmeans_.cluster(X, indices, k_);
    std::vector<std::vector<int> > children_indices(k_);
    std::vector<std::vector<int> > children_array_indices(k_);
    for (int i=0; i<n; i++)
    {
        children_indices[ current_cluster_result[i] ].push_back( indices[i] );
        children_array_indices[ current_cluster_result[i] ].push_back( i );
    }


    if (verbose_)
    {
        LOG("hierarchical k-means: [ %d ] -> [ ", indices.size());
        for (int i=0; i<k_; i++) LOG("%d ", children_indices[i].size());
        LOG("]\n");
    }


    int num_clusters = 0;
    for (int i=0; i<k_; i++)
    {
        std::vector<int> hierarchical_cluster_result = clusterSizeConstraint(X, children_indices[i]);

        std::map<int, int> remap;

        for (int j=0; j<hierarchical_cluster_result.size(); j++)
        {
            if (remap.find( hierarchical_cluster_result[j] ) == remap.end())
            {
                const int size = remap.size();
                remap[ hierarchical_cluster_result[j] ] = size;
            }

            result[ children_array_indices[i][j] ] = num_clusters + remap[ hierarchical_cluster_result[j] ];
        }

        num_clusters += remap.size();
    }

    return result;
}
