#include <ros/ros.h>

#include <pcpred/gaussian_process/gpr_multivariate.h>
#include <pcpred/util/gaussian_quadrature.h>
#include <pcpred/util/eigenmvn.h>

#include <visualization_msgs/MarkerArray.h>

#include <stdio.h>
#include <iostream>

using namespace pcpred;


Eigen::Vector3d hermite(const Eigen::Vector3d& p0,
               const Eigen::Vector3d& v0,
               const Eigen::Vector3d& p1,
               const Eigen::Vector3d& v1,
               double s)
{
    const double s2 = s * s;
    const double s3 = s2 * s;

    return p0 * (2*s3 - 3*s2 + 1) + v0 * (s3 - 2*s2 + s) + p1 * (-2*s3 + 3*s2) + v1 * (s3 - s2);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gpr_curves");
    ROS_INFO("gpr_curves");
    ros::NodeHandle nh;
    ros::Duration d(1.0);

    d.sleep();

    const int num_curves = 10;
    const int num_control_points = 4;
    Eigen::Vector3d mean_curve[] =
    {
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(0, 3, 0),
        Eigen::Vector3d(1, 1, 1),
        Eigen::Vector3d(0, 0, 3),
    };
    const int num_output_control_points = 4;
    Eigen::Vector3d output_mean_curve[] =
    {
        Eigen::Vector3d(2, 2, 2),
        Eigen::Vector3d(0, 3, 0),
    };
    const double covariance = 0.1;

    Eigen::Vector3d test_mean_curve[] =
    {
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(0, 3, 0),
        Eigen::Vector3d(1, 1, 1),
        Eigen::Vector3d(0, 0, 3),
    };
    const double test_covariance = 0.1;

    const double l = 0.01;

    const double gpr_l = 1.0;
    const double gpr_sigma_f = 10.0;
    const double gpr_sigma_n = std::sqrt(covariance);

    const int res = 16;

    Eigen::EigenMultivariateNormal<double> normal( Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity() * covariance );
    Eigen::EigenMultivariateNormal<double> prediction_normal( Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity() * test_covariance );

    // curve generation
    std::vector<std::vector<Eigen::Vector3d> > curves;
    std::vector<std::vector<Eigen::Vector3d> > output_curves;
    for (int i=0; i<num_curves; i++)
    {
        Eigen::MatrixXd e = normal.samples(num_control_points);

        std::vector<Eigen::Vector3d> curve;
        for (int j=0; j<num_control_points; j++)
            curve.push_back(mean_curve[j] + e.col(j));

        curves.push_back(curve);
    }
    for (int i=0; i<num_curves; i++)
    {
        Eigen::MatrixXd e = normal.samples(num_control_points);

        std::vector<Eigen::Vector3d> curve;
        curve.push_back(curves[i][num_control_points - 2]);
        curve.push_back(curves[i][num_control_points - 1]);
        for (int j=0; j<num_output_control_points - 2; j++)
            curve.push_back(output_mean_curve[j] + e.col(j));

        output_curves.push_back(curve);
    }

    // feature extraction
    Eigen::MatrixXd X( (num_control_points/2 - 1) * 12, num_curves );
    std::vector<Eigen::VectorXd> Y( num_output_control_points * 3 );
    for (int i=0; i<Y.size(); i++)
        Y[i].resize(num_curves);

    std::vector<double> gqx, gqw;
    getGaussianQuadratureCoefficients3(gqx, gqw);
    for (int i=0; i<num_curves; i++)
    {
        int idx = 0;
        for (int j=0; j<num_control_points/2 - 1; j++)
        {
            const Eigen::Vector3d p0 = curves[i][2*j+0];
            const Eigen::Vector3d v0 = curves[i][2*j+1];
            const Eigen::Vector3d p1 = curves[i][2*j+2];
            const Eigen::Vector3d v1 = curves[i][2*j+3];

            for (int k=0; k<4; k++)
            {
                Eigen::Vector3d p = 0.5 * gqw[k] * hermite(p0, v0, p1, v1, 0.5 + 0.5 * gqx[k]);
                X(idx++, i) = p(0);
                X(idx++, i) = p(1);
                X(idx++, i) = p(2);
            }
        }

        for (int j=1; j<output_curves[i].size(); j++)
        {
            const Eigen::Vector3d p = output_curves[i][j];
            Y[3*j + 0](i) = p(0);
            Y[3*j + 1](i) = p(1);
            Y[3*j + 2](i) = p(2);
        }
    }

    // training GPR
    std::vector<GprMultivariate> gpr( Y.size() );
    for (int i=0; i<gpr.size(); i++)
    {
        gpr[i].setHyperParameters(gpr_l, gpr_sigma_f, gpr_sigma_n);
        gpr[i].setObservation(X, Y[i]);
    }

    while (true)
    {
        // test curve generation
        Eigen::MatrixXd e = prediction_normal.samples(num_control_points);
        std::vector<Eigen::Vector3d> test_curve;
        for (int j=0; j<num_control_points; j++)
            test_curve.push_back(test_mean_curve[j] + e.col(j));

        // test curve feature extraction
        Eigen::VectorXd x( (num_control_points/2 - 1) * 12 );
        int idx = 0;
        for (int j=0; j<num_control_points/2 - 1; j++)
        {
            const Eigen::Vector3d p0 = test_curve[2*j+0];
            const Eigen::Vector3d v0 = test_curve[2*j+1];
            const Eigen::Vector3d p1 = test_curve[2*j+2];
            const Eigen::Vector3d v1 = test_curve[2*j+3];

            for (int k=0; k<4; k++)
            {
                Eigen::Vector3d p = 0.5 * gqw[k] * hermite(p0, v0, p1, v1, 0.5 + 0.5 * gqx[k]);
                x(idx++) = p(0);
                x(idx++) = p(1);
                x(idx++) = p(2);
            }
        }

        // regression
        std::vector<double> means;
        std::vector<double> covariances;
        for (int i=0; i<gpr.size(); i++)
        {
            double mean, covariance;
            gpr[i].regression(x, mean, covariance);
            means.push_back(mean);
            covariances.push_back(covariance);
        }

        // input curve visualize
        ros::Publisher publisher = nh.advertise<visualization_msgs::MarkerArray>("gpr_curves", 100);
        visualization_msgs::MarkerArray marker_array;
        for (int i=0; i<num_curves; i++)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/world";
            marker.header.stamp = ros::Time();
            marker.ns = "curve_input";
            marker.id = i;
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

            const int end = num_control_points/2 - 1;
            for (int j=0; j<end; j++)
            {
                const Eigen::Vector3d p0 = curves[i][2*j+0];
                const Eigen::Vector3d v0 = curves[i][2*j+1];
                const Eigen::Vector3d p1 = curves[i][2*j+2];
                const Eigen::Vector3d v1 = curves[i][2*j+3];

                geometry_msgs::Point point;
                std_msgs::ColorRGBA color;

                const int resend = j == end-1 ? res+1 : res;
                for (int k=0; k<resend; k++)
                {
                    const double t = (double)k / res;
                    Eigen::Vector3d h = hermite(p0, v0, p1, v1, t);

                    point.x = h(0);
                    point.y = h(1);
                    point.z = h(2);

                    const double ct = (j + t) / end;
                    color.r = color.g = color.b = ct;
                    color.a = 1.0;

                    marker.points.push_back(point);
                    marker.colors.push_back(color);
                }
            }

            marker_array.markers.push_back(marker);
        }

        // output curve visualize
        for (int i=0; i<num_curves; i++)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/world";
            marker.header.stamp = ros::Time();
            marker.ns = "curve_output";
            marker.id = i;
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

            const int end = num_output_control_points/2 - 1;
            for (int j=0; j<end; j++)
            {
                const Eigen::Vector3d p0 = output_curves[i][2*j+0];
                const Eigen::Vector3d v0 = output_curves[i][2*j+1];
                const Eigen::Vector3d p1 = output_curves[i][2*j+2];
                const Eigen::Vector3d v1 = output_curves[i][2*j+3];

                geometry_msgs::Point point;
                std_msgs::ColorRGBA color;

                const int resend = j == end-1 ? res+1 : res;
                for (int k=0; k<resend; k++)
                {
                    const double t = (double)k / res;
                    Eigen::Vector3d h = hermite(p0, v0, p1, v1, t);

                    point.x = h(0);
                    point.y = h(1);
                    point.z = h(2);

                    const double ct = (j + t) / end;
                    color.r = color.g = ct;
                    color.b = 1.0 - ct;
                    color.a = 1.0;

                    marker.points.push_back(point);
                    marker.colors.push_back(color);
                }
            }

            marker_array.markers.push_back(marker);
        }

        // test curve visualize
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/world";
        marker.header.stamp = ros::Time();
        marker.ns = "curve_test";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = l * 5.0;
        marker.pose.position.x = 0.;
        marker.pose.position.y = 0.;
        marker.pose.position.z = 0.;
        marker.pose.orientation.w = 1.;
        marker.pose.orientation.x = 0.;
        marker.pose.orientation.y = 0.;
        marker.pose.orientation.z = 0.;

        const int end = num_control_points/2 - 1;
        for (int j=0; j<end; j++)
        {
            const Eigen::Vector3d p0 = test_curve[2*j+0];
            const Eigen::Vector3d v0 = test_curve[2*j+1];
            const Eigen::Vector3d p1 = test_curve[2*j+2];
            const Eigen::Vector3d v1 = test_curve[2*j+3];

            geometry_msgs::Point point;
            std_msgs::ColorRGBA color;

            const int resend = j == end-1 ? res+1 : res;
            for (int k=0; k<resend; k++)
            {
                const double t = (double)k / res;
                Eigen::Vector3d h = hermite(p0, v0, p1, v1, t);

                point.x = h(0);
                point.y = h(1);
                point.z = h(2);

                const double ct = (j + t) / end;
                color.r = ct;
                color.g = 1.0 - ct;
                color.b = 0.0;
                color.a = 1.0;

                marker.points.push_back(point);
                marker.colors.push_back(color);
            }
        }
        marker_array.markers.push_back(marker);

        // regressed curve visualize
        marker.header.frame_id = "/world";
        marker.header.stamp = ros::Time();
        marker.ns = "curve_regression";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = l * 5.0;
        marker.pose.position.x = 0.;
        marker.pose.position.y = 0.;
        marker.pose.position.z = 0.;
        marker.pose.orientation.w = 1.;
        marker.pose.orientation.x = 0.;
        marker.pose.orientation.y = 0.;
        marker.pose.orientation.z = 0.;
        marker.points.clear();
        marker.colors.clear();

        const int r_end = num_control_points/2 - 1;
        for (int j=0; j<r_end; j++)
        {
            Eigen::Vector3d p0 = Eigen::Vector3d( means[(2*j+0)*3+0], means[(2*j+0)*3+1], means[(2*j+0)*3+2] );
            Eigen::Vector3d v0 = Eigen::Vector3d( means[(2*j+1)*3+0], means[(2*j+1)*3+1], means[(2*j+1)*3+2] );
            const Eigen::Vector3d p1 = Eigen::Vector3d( means[(2*j+2)*3+0], means[(2*j+2)*3+1], means[(2*j+2)*3+2] );
            const Eigen::Vector3d v1 = Eigen::Vector3d( means[(2*j+3)*3+0], means[(2*j+3)*3+1], means[(2*j+3)*3+2] );

            if (j==0)
            {
                p0 = test_curve[ num_control_points - 2];
                v0 = test_curve[ num_control_points - 1];
            }

            geometry_msgs::Point point;
            std_msgs::ColorRGBA color;

            const int resend = j == r_end-1 ? res+1 : res;
            for (int k=0; k<resend; k++)
            {
                const double t = (double)k / res;
                Eigen::Vector3d h = hermite(p0, v0, p1, v1, t);

                point.x = h(0);
                point.y = h(1);
                point.z = h(2);

                const double ct = (j + t) / r_end;
                color.r = 0.5;
                color.g = ct;
                color.b = 1.0 - ct;
                color.a = 1.0;

                marker.points.push_back(point);
                marker.colors.push_back(color);
            }
        }
        marker_array.markers.push_back(marker);

        publisher.publish(marker_array);

        d.sleep();
    }

    return 0;
}
