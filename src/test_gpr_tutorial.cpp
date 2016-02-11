#include <ros/ros.h>

#include <pcpred/gaussian_process/gpr.h>

#include <stdio.h>

using namespace pcpred;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "gpr_tutorial");
    ROS_INFO("gpr_tutorial");
    ros::NodeHandle nh;
    ros::Duration d(1.0);

    GaussianProcessRegression gpr;

    d.sleep();

    const int n = 100;
    const double m = -1.7;
    const double M = 0.3;

    Eigen::VectorXd X(6), Y(6), O(n);
    X << -1.50, -1.00, -0.75, -0.40, -0.25, 0.00;
    Y << -1.70, -1.20, -0.40, 0.10, 0.40, 0.70;

    for (int i=0; i<n; i++)
    {
        const double t = (double)i / (n-1);
        O(i) = (1-t) * m + t * M;
    }

    gpr.setObservation(X, Y);
    gpr.visualizeRegression(O);

    d.sleep();

    return 0;
}
