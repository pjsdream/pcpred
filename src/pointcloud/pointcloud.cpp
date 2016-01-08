#include <pcpred/pointcloud/pointcloud.h>

#include <Eigen/Geometry>

#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace pcpred;


Pointcloud::Pointcloud()
{
}

Pointcloud::Pointcloud(const std::vector<Eigen::Vector3d>& pointcloud_list)
{
    pointcloud_ = pointcloud_list;
}


void Pointcloud::push_back(const Eigen::Vector3d& point)
{
    pointcloud_.push_back(point);
}

void Pointcloud::rotate(double angle, const Eigen::Vector3d& axis)
{
    Eigen::AngleAxisd aa(angle, axis);

    for (int i=0; i<pointcloud_.size(); i++)
        pointcloud_[i] = aa * pointcloud_[i];
}

Pointcloud Pointcloud::cluster(double voxel_resolution, double seed_resolution)
{
    Pointcloud result;
    if (pointcloud_.empty())
        return result;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pointcloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i=0; i<pointcloud_.size(); i++)
    {
        pcl::PointXYZRGB point(pointcloud_[i](0), pointcloud_[i](1), pointcloud_[i](2));
        point.x = pointcloud_[i](0);
        point.y = pointcloud_[i](1);
        point.z = pointcloud_[i](2);
        pcl_pointcloud->push_back(point);
    }

    pcl::SupervoxelClustering<pcl::PointXYZRGB> super(voxel_resolution, seed_resolution);
    super.setInputCloud(pcl_pointcloud);
    super.setColorImportance(0.0f);
    super.setSpatialImportance(0.4f);
    super.setNormalImportance(1.0f);

    std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr> supervoxel_clusters;
    super.extract(supervoxel_clusters);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr extracted_cloud = super.getVoxelCentroidCloud();

    for (int i=0; i<extracted_cloud->size(); i++)
    {
        const pcl::PointXYZRGB& point = extracted_cloud->at(i);
        result.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }

    return result;
}
