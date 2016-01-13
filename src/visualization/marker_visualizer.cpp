#include <pcpred/visualization/marker_visualizer.h>

#include <Eigen/SVD>

#include <cmath>

#include <stdio.h>
#include <iostream>

#include <string>

using namespace pcpred;


std::map<double, double> MarkerVisualizer::gaussian_distribution_radius_table_;
bool MarkerVisualizer::gaussian_distribution_radius_table_initialized_ = false;


MarkerVisualizer::MarkerVisualizer(const char* topic)
    : Visualizer(topic)
{
    if (gaussian_distribution_radius_table_initialized_ == false)
    {
        // from chi-squared distribution wiki page
        // dimension = 3
        gaussian_distribution_radius_table_initialized_ = true;
        gaussian_distribution_radius_table_[1.0 - 0.95 ] = std::sqrt( 0.35);
        gaussian_distribution_radius_table_[1.0 - 0.90 ] = std::sqrt( 0.58);
        gaussian_distribution_radius_table_[1.0 - 0.80 ] = std::sqrt( 1.01);
        gaussian_distribution_radius_table_[1.0 - 0.70 ] = std::sqrt( 1.42);
        gaussian_distribution_radius_table_[1.0 - 0.50 ] = std::sqrt( 2.37);
        gaussian_distribution_radius_table_[1.0 - 0.30 ] = std::sqrt( 3.66);
        gaussian_distribution_radius_table_[1.0 - 0.20 ] = std::sqrt( 4.64);
        gaussian_distribution_radius_table_[1.0 - 0.10 ] = std::sqrt( 6.25);
        gaussian_distribution_radius_table_[1.0 - 0.05 ] = std::sqrt( 7.82);
        gaussian_distribution_radius_table_[1.0 - 0.01 ] = std::sqrt(11.34);
        gaussian_distribution_radius_table_[1.0 - 0.001] = std::sqrt(16.27);
    }

    ros::NodeHandle n;
    publisher_ = n.advertise<visualization_msgs::Marker>(topic, CAPACITY);
}


void MarkerVisualizer::publish(const visualization_msgs::Marker& marker)
{
    if (publisher_.getNumSubscribers() < 1)
    {
        if (!ros::ok())
            return;
        ROS_WARN_ONCE("Please create a subscriber to the marker");
    }
    publisher_.publish(marker);
}


void MarkerVisualizer::drawSphere(const char* ns, int id, const Eigen::Vector3d& center, double radius)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();

    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = center[0];
    marker.pose.position.y = center[1];
    marker.pose.position.z = center[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    publish(marker);
}

void MarkerVisualizer::drawEllipsoid(const char* ns, int id, const Eigen::Vector3d& center, const Eigen::Matrix3d& A)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();

    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    // axis: eigenvectors
    // radius: eigenvalues
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd& r = svd.singularValues();
    Eigen::Matrix3d Q = svd.matrixU();

    // to make determinant 1
    if (Q.determinant() < 0)
        Q.col(2) *= -1.;
    const Eigen::Quaterniond q(Q);

    marker.pose.position.x = center(0);
    marker.pose.position.y = center(1);
    marker.pose.position.z = center(2);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = r(0);
    marker.scale.y = r(1);
    marker.scale.z = r(2);

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;

    marker.lifetime = ros::Duration();

    publish(marker);
}

void MarkerVisualizer::drawGaussianDistribution(const char* ns, int id, const Eigen::Vector3d& mu, const Eigen::Matrix3d& sigma, double probability, double offset)
{
    if (gaussian_distribution_radius_table_.find(probability) == gaussian_distribution_radius_table_.end())
    {
        ROS_WARN("gaussian distribution drawing for probability %lf is not supported", probability);
        ROS_WARN("supported probabilities: 0.05 0.1 0.2 0.3 0.5 0.7 0.8 0.9 0.95 0.99 0.999");
        return;
    }

    visualization_msgs::Marker marker;

    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();

    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    // axis: eigenvectors of covariance
    // radius: square roots of eigenvalues of covariance
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(sigma, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd& r = svd.singularValues();
    Eigen::Matrix3d Q = svd.matrixU();

    // to make determinant 1
    if (Q.determinant() < 0)
        Q.col(2) *= -1.;
    const Eigen::Quaterniond q(Q);

    const double probability_radius = gaussian_distribution_radius_table_[probability];

    marker.pose.position.x = mu[0];
    marker.pose.position.y = mu[1];
    marker.pose.position.z = mu[2];
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = probability_radius * std::sqrt(r[0]) + offset;
    marker.scale.y = probability_radius * std::sqrt(r[1]) + offset;
    marker.scale.z = probability_radius * std::sqrt(r[2]) + offset;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;

    marker.lifetime = ros::Duration();

    publish(marker);
}

void MarkerVisualizer::clear(const char* ns, int id)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();

    marker.ns = ns;
    marker.id = id;
    marker.action = visualization_msgs::Marker::DELETE;

    publish(marker);
}

void MarkerVisualizer::clearUptoCapacity(const char* ns)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();

    marker.ns = ns;
    marker.action = visualization_msgs::Marker::DELETE;

    for (int i=0; i<CAPACITY; i++)
    {
        marker.id = i;
        publish(marker);
    }
}
