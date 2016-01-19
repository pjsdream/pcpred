#include <pcpred/visualization/marker_array_visualizer.h>

#include <Eigen/SVD>

#include <cmath>

using namespace pcpred;


MarkerArrayVisualizer::MarkerArrayVisualizer(const char* topic)
    : Visualizer(topic)
{
    ros::NodeHandle n;
    publisher_ = n.advertise<visualization_msgs::MarkerArray>(topic, CAPACITY);

    ros::Rate delay(1.0);
    delay.sleep();
}


void MarkerArrayVisualizer::publish(const visualization_msgs::MarkerArray& marker)
{
    if (publisher_.getNumSubscribers() < 1)
    {
        if (!ros::ok())
            return;
        ROS_WARN_ONCE("Please create a subscriber to the marker array");
    }
    publisher_.publish(marker);
}


void MarkerArrayVisualizer::drawSphere(const char* ns, const Eigen::Vector3d& center, double radius)
{
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker marker;

    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();

    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = center(0);
    marker.pose.position.y = center(1);
    marker.pose.position.z = center(2);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 2. * radius;
    marker.scale.y = 2. * radius;
    marker.scale.z = 2. * radius;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    marker_array.markers.push_back(marker);

    publish(marker_array);
}

void MarkerArrayVisualizer::drawSpheres(const char* ns, const std::vector<Eigen::Vector3d>& center, const std::vector<double> radius)
{
    visualization_msgs::MarkerArray marker_array;

    const int size = center.size();

    for (int i=0; i<size; i++)
    {
        visualization_msgs::Marker marker;

        marker.header.frame_id = "/world";
        marker.header.stamp = ros::Time::now();

        marker.ns = ns;
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = center[i](0);
        marker.pose.position.y = center[i](1);
        marker.pose.position.z = center[i](2);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 2. * radius[i];
        marker.scale.y = 2. * radius[i];
        marker.scale.z = 2. * radius[i];

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        marker_array.markers.push_back(marker);
    }

    publish(marker_array);
}

void MarkerArrayVisualizer::drawEllipsoids(const char* ns, const std::vector<Eigen::Vector3d>& center, const std::vector<Eigen::Matrix3d>& A)
{
    visualization_msgs::MarkerArray marker_array;

    const int size = center.size();

    for (int i=0; i<size; i++)
    {
        visualization_msgs::Marker marker;

        marker.header.frame_id = "/world";
        marker.header.stamp = ros::Time::now();

        marker.ns = ns;
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        // axis: eigenvectors
        // radius: eigenvalues
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A[i], Eigen::ComputeThinU | Eigen::ComputeThinV);
        const Eigen::VectorXd& r = svd.singularValues();
        Eigen::Matrix3d Q = svd.matrixU();

        // to make determinant 1
        if (Q.determinant() < 0)
            Q.col(2) *= -1.;
        const Eigen::Quaterniond q(Q);

        marker.pose.position.x = center[i](0);
        marker.pose.position.y = center[i](1);
        marker.pose.position.z = center[i](2);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        marker.scale.x = 2. * r(0);
        marker.scale.y = 2. * r(1);
        marker.scale.z = 2. * r(2);

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;

        marker.lifetime = ros::Duration();

        marker_array.markers.push_back(marker);
    }

    publish(marker_array);
}

void MarkerArrayVisualizer::drawSphereList(const char* ns, const std::vector<Eigen::Vector3d>& centers, double radius)
{
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker marker;

    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();

    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 2. * radius;
    marker.scale.y = 2. * radius;
    marker.scale.z = 2. * radius;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    const int size = centers.size();

    for (int i=0; i<size; i++)
    {
        ::geometry_msgs::Point point;
        point.x = centers[i](0);
        point.y = centers[i](1);
        point.z = centers[i](2);
        marker.points.push_back(point);
    }

    marker_array.markers.push_back(marker);
    publish(marker_array);
}

void MarkerArrayVisualizer::drawLineStrip(const char* ns, const std::vector<Eigen::Vector3d>& endpoints)
{
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker marker;

    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();

    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.01;
    marker.scale.y = 0.0;
    marker.scale.z = 0.0;

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    const int size = endpoints.size();
    geometry_msgs::Point point;

    for (int i=0; i<size; i++)
    {
        point.x = endpoints[i](0);
        point.y = endpoints[i](1);
        point.z = endpoints[i](2);
        marker.points.push_back(point);
    }

    marker_array.markers.push_back(marker);
    publish(marker_array);
}

void MarkerArrayVisualizer::drawLineList(const char* ns, const std::vector<Eigen::Vector3d>& endpoints)
{
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker marker;

    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();

    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.01;
    marker.scale.y = 0.0;
    marker.scale.z = 0.0;

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    const int size = endpoints.size();
    geometry_msgs::Point point;

    for (int i=0; i<size; i++)
    {
        point.x = endpoints[i](0);
        point.y = endpoints[i](1);
        point.z = endpoints[i](2);
        marker.points.push_back(point);
    }

    marker_array.markers.push_back(marker);
    publish(marker_array);
}
