#ifndef POINTCLOUD_H
#define POINTCLOUD_H


#include <Eigen/Dense>

#include <vector>


namespace pcpred
{

class Pointcloud
{
public:

    Pointcloud();
    Pointcloud(const std::vector<Eigen::Vector3d>& pointcloud_list);

    inline int size() const { return pointcloud_.size(); }
    inline const Eigen::Vector3d& point(int i) const { return pointcloud_[i]; }

    void push_back(const Eigen::Vector3d& point);
    void rotate(double angle, const Eigen::Vector3d& axis);

    Pointcloud cluster(double voxel_resolution, double seed_resolution);

private:

    std::vector<Eigen::Vector3d> pointcloud_;
};

}

#endif // POINTCLOUD_H
