#include <pcpred/pointcloud/pointcloud.h>

#include <Eigen/Geometry>

#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace pcpred;


Pointcloud::Pointcloud(const std::vector<Eigen::Vector3d>& pointcloud_list)
{
    pointcloud_ = pointcloud_list;
}


void Pointcloud::rotate(double angle, const Eigen::Vector3d& axis)
{
    Eigen::AngleAxisd aa(angle, axis);

    for (int i=0; i<pointcloud_.size(); i++)
        pointcloud_[i] = aa * pointcloud_[i];
}

void Pointcloud::cluster(double voxel_resolution, double seed_resolution)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i=0; i<pointcloud_.size(); i++)
        pcl_pointcloud->push_back(pcl::PointXYZ(pointcloud_[i](0), pointcloud_[i](1), pointcloud_[i](2)));

    pcl::SupervoxelClustering<pcl::PointXYZ> super(voxel_resolution, seed_resolution);
    super.setInputCloud(pcl_pointcloud);
    super.setColorImportance(0.0f);
    super.setSpatialImportance(0.4f);
    super.setNormalImportance(1.0f);

    std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZ>::Ptr> supervoxel_clusters;
    super.extract(supervoxel_clusters);
}
