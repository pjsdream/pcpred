#include <pcpred/learning/hierarchical_kmeans.h>

#include <pcpred/util/eigenmvn.h>

#include <visualization_msgs/MarkerArray.h>

#include <ros/ros.h>

#include <cstdlib>

using namespace pcpred;


int main(int argc, char** argv)
{
    srand(time(NULL));

    ros::init(argc, argv, "hierarchical_kmeans");
    ROS_INFO("hierarchical_kmeans");
    ros::NodeHandle nh;
    ros::Duration d(1.0);

    const int n = 100000;
    const int k = 2;
    const int max_clusters = 10000;
    const int size_limit = 100;
    const double covariance = 1.0;
    const double r = 0.01;

    HierarchicalKmeans kmeans;
    kmeans.setK(k);
    kmeans.setSizeLimit(size_limit);

    Eigen::EigenMultivariateNormal<double> normal( Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity() * covariance );

    ros::Publisher publisher = nh.advertise<visualization_msgs::MarkerArray>("hierarchical_kmeans", 100);

    d.sleep();

    int num_clusters = 0;
    int previous_num_clusters = 0;
    while (ros::ok())
    {
        Eigen::MatrixXd p = normal.samples(n);

        const double start_time = ros::Time::now().toSec();
        std::vector<int> cluster = kmeans.clusterSizeConstraint(p);
        printf("clustering time: %lf\n", ros::Time::now().toSec() - start_time);
        fflush(stdout);

        num_clusters = 0;
        for (int i=0; i<cluster.size(); i++)
        {
            if (num_clusters < cluster[i])
                num_clusters = cluster[i];
        }
        num_clusters++;

        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;

        marker.header.stamp = ros::Time();
        marker.header.frame_id = "/world";
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.;
        marker.pose.position.y = 0.;
        marker.pose.position.z = 0.;
        marker.pose.orientation.w = 1.;
        marker.pose.orientation.x = 0.;
        marker.pose.orientation.y = 0.;
        marker.pose.orientation.z = 0.;
        marker.scale.x = r * 2.;
        marker.scale.y = r * 2.;
        marker.scale.z = r * 2.;

        for (int i=0; i<num_clusters; i++)
        {
            marker.ns = "cluster";
            marker.id = i;
            marker.color.r = (double)rand() / RAND_MAX;
            marker.color.g = (double)rand() / RAND_MAX;
            marker.color.b = (double)rand() / RAND_MAX;
            marker.color.a = 1.0;

            marker.points.clear();
            for (int j=0; j<cluster.size(); j++)
            {
                if (cluster[j] == i)
                {
                    geometry_msgs::Point point;
                    point.x = p(0, j);
                    point.y = p(1, j);
                    point.z = p(2, j);
                    marker.points.push_back(point);
                }
            }
            marker_array.markers.push_back(marker);
        }

        marker.action = visualization_msgs::Marker::DELETE;
        for (int i=num_clusters; i<previous_num_clusters; i++)
        {
            marker.ns = "cluster";
            marker.id = i;
            marker_array.markers.push_back(marker);
        }

        publisher.publish(marker_array);

        previous_num_clusters = num_clusters;

        d.sleep();
    }

    return 0;
}
