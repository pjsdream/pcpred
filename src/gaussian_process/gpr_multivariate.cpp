#include <pcpred/gaussian_process/gpr_multivariate.h>

#include <visualization_msgs/MarkerArray.h>

#include <stdio.h>
#include <iostream>

using namespace pcpred;


GprMultivariate::GprMultivariate()
{
    setHyperParameters(1.0, 1.27, 0.3);

    setVisualizerTopic("gpr_curves");
}


void GprMultivariate::setVisualizerTopic(const std::string &topic)
{
    ros::NodeHandle n;
    publisher_ = n.advertise<visualization_msgs::MarkerArray>(topic, 1000);
}

double GprMultivariate::kernel(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2)
{
    return sigma_f_ * sigma_f_ * std::exp( -(x1-x2).dot(x1-x2) / (2. * l_ * l_));
}

void GprMultivariate::setObservation(const Eigen::MatrixXd &X, const Eigen::MatrixXd& Y)
{
    setObservationInput(X);
    setObservationOutput(Y);
}

void GprMultivariate::setObservationInput(const Eigen::MatrixXd &X)
{
    X_ = X;

    // covariance matrix
    const int n = X_.cols();
    K_.resize(n, n);
    for (int i=0; i<n; i++)
    {
        for (int j=i; j<n; j++)
        {
            K_(i,j) = kernel(X.col(i), X.col(j));

            if (i==j)
                K_(i,j) += sigma_n_ * sigma_n_;

            K_(j,i) = K_(i,j);
        }
    }

    K_inverse_ = K_.inverse();
}

void GprMultivariate::setObservationOutput(const Eigen::VectorXd &Y)
{
    Y_ = Y;
}

void GprMultivariate::setHyperParameters(double l, double sigma_f, double sigma_n)
{
    l_ = l;
    sigma_f_ = sigma_f;
    sigma_n_ = sigma_n;
}

void GprMultivariate::print()
{
    std::cout << "K = " << std::endl << K_ << std::endl;
}

void GprMultivariate::regression(const Eigen::MatrixXd& X, Eigen::VectorXd& mean, Eigen::MatrixXd& variance)
{
    const int n = X_.cols();
    const int m = X.cols();

    Eigen::MatrixXd K1(m, n);
    Eigen::MatrixXd K2(m, m);

    for (int i=0; i<m; i++)
    {
        for (int j=0; j<n; j++)
        {
            K1(i,j) = kernel(X.col(i), X_.col(j));
        }
    }

    for (int i=0; i<m; i++)
    {
        for (int j=i; j<m; j++)
        {
            K2(i,j) = kernel(X.col(i), X.col(j));

            if (i==j)
                K2(i,j) += sigma_n_ * sigma_n_;

            K2(j,i) = K2(i,j);
        }
    }

    mean = K1 * (K_inverse_ * Y_);
    variance = K2 - K1 * K_inverse_ * K1.transpose();
}

void GprMultivariate::regression(const Eigen::VectorXd& x, double& mean, double& variance)
{
    const int n = X_.cols();
    const int m = 1;

    Eigen::MatrixXd K1(m, n);
    Eigen::MatrixXd K2(m, m);

    for (int i=0; i<m; i++)
    {
        for (int j=0; j<n; j++)
        {
            K1(i,j) = kernel(x, X_.col(j));
        }
    }

    for (int i=0; i<m; i++)
    {
        for (int j=i; j<m; j++)
        {
            K2(i,j) = kernel(x, x);

            if (i==j)
                K2(i,j) += sigma_n_ * sigma_n_;

            K2(j,i) = K2(i,j);
        }
    }

    mean = (K1 * (K_inverse_ * Y_))(0,0);
    variance = (K2 - K1 * K_inverse_ * K1.transpose())(0,0);
}

void GprMultivariate::visualizeRegression(const Eigen::MatrixXd& X)
{
    Eigen::VectorXd mean;
    Eigen::MatrixXd variance;

    regression(X, mean, variance);

    const double r = 0.05;
    const double l = 0.01;
    const int n = X_.rows();
    const int m = X.rows();

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time();
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;
    marker.pose.orientation.w = 1.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;
    marker.scale.x = r;
    marker.scale.y = r;
    marker.scale.z = r;
    marker.color.a = 1.;

    // input
    /*
    marker.ns = "input";
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.id = 0;
    marker.color.r = 0.;
    marker.color.g = 0.;
    marker.color.b = 1.;
    for (int i=0; i<n; i++)
    {
        geometry_msgs::Point p;
        p.x = X_[i];
        p.y = Y_[i];
        p.z = 0.;
        marker.points.push_back(p);
    }
    marker_array.markers.push_back(marker);

    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.id = 1;
    marker.scale.x = l;
    marker.points.clear();
    for (int i=0; i<n; i++)
    {
        geometry_msgs::Point p;
        p.x = X_[i];
        p.y = Y_[i] - sigma_n_;
        p.z = 0.;
        marker.points.push_back(p);

        p.x = X_[i];
        p.y = Y_[i] + sigma_n_;
        p.z = 0.;
        marker.points.push_back(p);
    }
    marker_array.markers.push_back(marker);
    */

    // regression
    /*
    marker.ns = "regression";
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.id = 0;
    marker.color.r = 1.;
    marker.color.g = 0.;
    marker.color.b = 0.;
    marker.scale.x = r;
    marker.points.clear();
    for (int i=0; i<m; i++)
    {
        geometry_msgs::Point p;
        p.x = X[i];
        p.y = mean[i];
        p.z = 0.;
        marker.points.push_back(p);
    }
    marker_array.markers.push_back(marker);

    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.id = 1;
    marker.scale.x = l;
    marker.points.clear();
    for (int i=0; i<m; i++)
    {
        const double stddev = std::sqrt(variance(i,i));
        geometry_msgs::Point p;
        p.x = X[i];
        p.y = mean[i] - 1.96 * stddev;
        p.z = 0.;
        marker.points.push_back(p);

        p.x = X[i];
        p.y = mean[i] + 1.96 * stddev;
        p.z = 0.;
        marker.points.push_back(p);
    }
    marker_array.markers.push_back(marker);
    */

    publisher_.publish(marker_array);
}
