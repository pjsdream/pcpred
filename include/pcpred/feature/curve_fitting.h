#ifndef CURVE_FITTING_H
#define CURVE_FITTING_H


#include <pcpred/feature/hermite_curve.h>

#include <Eigen/Dense>

#include <vector>

#include <ros/ros.h>


namespace pcpred
{

class CurveFitting
{
public:

    CurveFitting();

    void setCurveShape(int num_pieces);

    void fit(const std::vector<double>& t, const std::vector<Eigen::Vector3d>& x);

    Eigen::Vector3d curve(double t);

    void setVisualizerTopic(const std::string& topic);
    void visualizeCurve(int id = 0);

    HermiteCurve toHermiteCurve();

private:

    int n_;
    std::vector<Eigen::Vector3d> p_;
    std::vector<Eigen::Vector3d> v_;

    ros::Publisher publisher_;
};

}


#endif // CURVE_FITTING_H
