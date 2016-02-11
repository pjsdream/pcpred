#include <pcpred/feature/curve_fitting.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>

using namespace pcpred;


CurveFitting::CurveFitting()
{
    setVisualizerTopic("curve");

    setCurveShape(1);
}

void CurveFitting::setVisualizerTopic(const std::string& topic)
{
    ros::NodeHandle n;
    publisher_ = n.advertise<visualization_msgs::MarkerArray>(topic, 1000);
}

void CurveFitting::setCurveShape(int num_curves)
{
    n_ = num_curves;
    p_.resize(n_ + 1);
    v_.resize(n_ + 1);
}

void CurveFitting::fit(const std::vector<double> &t, const std::vector<Eigen::Vector3d> &x)
{
    const int m = x.size();

    Eigen::MatrixXd A(m, 2 * (n_ + 1));
    Eigen::VectorXd b(m);

    A.setZero();
    for (int i=0; i<m; i++)
    {
        int j = (int)t[i];
        if (j == n_) j--;

        const double s = t[i] - j;
        const double s2 = s * s;
        const double s3 = s2 * s;

        A(i, 2*j  ) = 2*s3 - 3*s2 + 1;
        A(i, 2*j+1) = s3 - 2*s2 + s;
        A(i, 2*j+2) = -2*s3 + 3*s2;
        A(i, 2*j+3) = s3 - s2;
    }

    const Eigen::MatrixXd AT = A.transpose();
    const Eigen::MatrixXd ATA_inv = (AT * A).inverse();

    for (int c=0; c<3; c++)
    {
        for (int i=0; i<m; i++)
            b(i) = x[i](c);

        Eigen::VectorXd x = ATA_inv * (AT * b);

        for (int i=0; i<=n_; i++)
        {
            p_[i](c) = x[2*i];
            v_[i](c) = x[2*i+1];
        }
    }
}

Eigen::Vector3d CurveFitting::curve(double t)
{
    int i = (int)t;
    if (i == n_) i--;

    const double s = t-i;
    const double s2 = s * s;
    const double s3 = s2 * s;

    /* Hermite spline basis
     * p0 : 2t^3 - 3t^2 + 1
     * v0 : t^3 - 2t^2 + t
     * p1 : -2t^3 + 3t^2
     * v1 : t^3 - t^2
     */
    return p_[i] * (2*s3 - 3*s2 + 1) + v_[i] * (s3 - 2*s2 + s) + p_[i+1] * (-2*s3 + 3*s2) + v_[i+1] * (s3 - s2);
}

void CurveFitting::visualizeCurve(int id)
{
    const double l = 0.01;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time();
    marker.id = id;
    marker.ns = "curve";
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = l;
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;
    marker.pose.orientation.w = 1.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;

    const int d = 16;
    const int m = d * n_ + 1;

    for (int i=0; i<m; i++)
    {
        const double t = (double)i / (m-1) * n_;

        const Eigen::Vector3d p = curve(t);
        geometry_msgs::Point point;
        point.x = p(0);
        point.y = p(1);
        point.z = p(2);
        marker.points.push_back(point);

        std_msgs::ColorRGBA color;
        color.r = color.g = color.b = (double)i / (m-1);
        color.a = 1.0;
        marker.colors.push_back(color);
    }
    marker_array.markers.push_back(marker);

    publisher_.publish(marker_array);
}

