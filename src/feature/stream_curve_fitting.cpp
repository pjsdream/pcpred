#include <pcpred/feature/stream_curve_fitting.h>

#include <vector>

#include <stdio.h>

using namespace pcpred;


StreamCurveFitting::StreamCurveFitting()
{
    setVisualizerTopic("curve");

    setCurveShape(1, 1.0);
}

void StreamCurveFitting::setVisualizerTopic(const std::string& topic)
{
    curve_.setVisualizerTopic(topic);
}

void StreamCurveFitting::push(double time, const Eigen::Vector3d& x)
{
    Data d =
    {
        time,
        x
    };

    data_.insert(d);

    while (!data_.empty() && time - data_.begin()->t > duration_)
        data_.erase(data_.begin());
}

void StreamCurveFitting::setCurveShape(int num_curves, double duration)
{
    n_ = num_curves;
    curve_.setCurveShape(num_curves);

    duration_ = duration;
}

void StreamCurveFitting::fit()
{
    if (data_.size() <= 10)
        return;

    std::vector<double> tlist;
    std::vector<Eigen::Vector3d> xlist;

    const double last_time = data_.rbegin()->t;

    for (std::set<Data>::iterator it = data_.begin(); it != data_.end(); it++)
    {
        tlist.push_back((it->t - last_time + duration_) / duration_ * n_);
        xlist.push_back(it->x);
    }

    curve_.fit(tlist, xlist);
}

Eigen::Vector3d StreamCurveFitting::curve(double t)
{
    return curve_.curve(t);
}

void StreamCurveFitting::visualizeCurve(int id)
{
    if (data_.size() <= 10)
        return;

    curve_.visualizeCurve(id);
}

