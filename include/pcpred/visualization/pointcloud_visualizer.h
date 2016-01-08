#ifndef POINTCLOUD_VISUALIZER_H
#define POINTCLOUD_VISUALIZER_H


#include <pcpred/visualization/visualizer.h>

#include <Eigen/Dense>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <pcpred/pointcloud/pointcloud.h>


namespace pcpred
{

class PointcloudVisualizer : public Visualizer
{
public:

    explicit PointcloudVisualizer(const char* topic = "pointcloud");

    void drawPointcloud(const std::vector<Eigen::Vector3d>& pointcloud);
    void drawPointcloud(const Pointcloud& pointcloud);

private:

    void publish(const pcl::PointCloud<pcl::PointXYZ>& marker);
};

}


#endif // POINTCLOUD_VISUALIZER_H
