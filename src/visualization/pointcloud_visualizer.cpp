#include <pcpred/visualization/pointcloud_visualizer.h>

using namespace pcpred;


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


PointcloudVisualizer::PointcloudVisualizer(const char* topic)
    : Visualizer(topic)
{
    ros::NodeHandle n;
    publisher_ = n.advertise<PointCloud>(topic, 1000);

    ros::Rate delay(1.0);
    delay.sleep();
}


void PointcloudVisualizer::publish(const PointCloud& marker)
{
    if (publisher_.getNumSubscribers() < 1)
    {
        if (!ros::ok())
            return;
        ROS_WARN_ONCE("Please create a subscriber to the pointcloud");
    }
    publisher_.publish(marker);
}

void PointcloudVisualizer::drawPointcloud(const std::vector<Eigen::Vector3d>& pointcloud)
{
    PointCloud msg;
    msg.header.frame_id = "/world";
    pcl_conversions::toPCL(ros::Time::now(), msg.header.stamp);

    msg.width = pointcloud.size();
    msg.height = 1;

    for (int i=0; i<pointcloud.size(); i++)
        msg.points.push_back(pcl::PointXYZ(pointcloud[i](0), pointcloud[i](1), pointcloud[i](2)));

    publisher_.publish(msg);
}

void PointcloudVisualizer::drawPointcloud(const Pointcloud& pointcloud)
{
    PointCloud msg;
    msg.header.frame_id = "/world";
    pcl_conversions::toPCL(ros::Time::now(), msg.header.stamp);

    msg.width = pointcloud.size();
    msg.height = 1;

    for (int i=0; i<pointcloud.size(); i++)
        msg.points.push_back(pcl::PointXYZ(pointcloud.point(i)(0), pointcloud.point(i)(1), pointcloud.point(i)(2)));

    publisher_.publish(msg);
}
