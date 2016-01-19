#include <pcpred/visualization/marker_visualizer.h>

#include <pcpred/util/chi_square.h>

#include <Eigen/SVD>

#include <cmath>

#include <stdio.h>
#include <iostream>

#include <string>

using namespace pcpred;


MarkerVisualizer::MarkerVisualizer(const char* topic)
    : Visualizer(topic)
{
    ros::NodeHandle n;
    publisher_ = n.advertise<visualization_msgs::Marker>(topic, CAPACITY);

    ros::Rate delay(1.0);
    delay.sleep();
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

    marker.scale.x = 2. * radius;
    marker.scale.y = 2. * radius;
    marker.scale.z = 2. * radius;

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

    marker.scale.x = 2. * r(0);
    marker.scale.y = 2. * r(1);
    marker.scale.z = 2. * r(2);

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;

    marker.lifetime = ros::Duration();

    publish(marker);
}

void MarkerVisualizer::drawGaussianDistribution(const char* ns, int id, const Eigen::Vector3d& mu, const Eigen::Matrix3d& sigma, double probability, double offset)
{
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

    const double probability_radius = gaussianDistributionRadius3D(probability);

    marker.pose.position.x = mu[0];
    marker.pose.position.y = mu[1];
    marker.pose.position.z = mu[2];
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = 2. * (probability_radius * std::sqrt(r[0]) + offset);
    marker.scale.y = 2. * (probability_radius * std::sqrt(r[1]) + offset);
    marker.scale.z = 2. * (probability_radius * std::sqrt(r[2]) + offset);

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
