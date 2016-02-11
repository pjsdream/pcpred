#ifndef STREAM_CURVE_FITTING_H
#define STREAM_CURVE_FITTING_H


#include <Eigen/Dense>

#include <pcpred/feature/curve_fitting.h>

#include <set>

#include <ros/ros.h>


namespace pcpred
{

class StreamCurveFitting
{
private:
    struct Data
    {
        double t;
        Eigen::Vector3d x;

        inline bool operator < (const Data& rhs) const
        {
            return t < rhs.t;
        }
    };

public:

    StreamCurveFitting();

    void setCurveShape(int num_curves, double duration);

    void push(double time, const Eigen::Vector3d& x);
    void fit();

    Eigen::Vector3d curve(double t);

    void setVisualizerTopic(const std::string& topic);
    void visualizeCurve(int id = 0);

private:

    int n_;
    double duration_;

    std::set<Data> data_;

    CurveFitting curve_;
};

}


#endif // STREAM_CURVE_FITTING_H
