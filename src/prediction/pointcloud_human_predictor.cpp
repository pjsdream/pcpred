#include <pcpred/prediction/pointcloud_human_predictor.h>

using namespace pcpred;


void PointcloudHumanPredictor::observe(const Pointcloud& pointcloud)
{
    std::vector<Eigen::Vector3d> points;
    for (int i=0; i<pointcloud.size(); i++)
        points.push_back(pointcloud.point(i));

    observe(points);
}

void PointcloudHumanPredictor::observe(const std::vector<Eigen::Vector3d>& pointcloud)
{
    optimizeHumanShape(pointcloud);
}

void PointcloudHumanPredictor::optimizeHumanShape(const std::vector<Eigen::Vector3d>& pointcloud)
{
    int iteration = 0;

    while (iteration < max_iteration_)
    {
        // TODO: optimize
        // solve for c

        // solve for x

        // length constraint adjustment

        iteration++;
    }
}
