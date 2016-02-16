#include <pcpred/feature/stream_curve_fitting.h>
#include <pcpred/util/gaussian_quadrature.h>

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

HermiteCurve StreamCurveFitting::toHermiteCurve()
{
    return curve_.toHermiteCurve();
}

void StreamCurveFitting::clear()
{
    data_.clear();
}

void StreamCurveFitting::push(double time, const Eigen::Vector3d& x)
{
    Data d =
    {
        time,
        x
    };

    data_.insert(d);

    while (!data_.empty() && data_.rbegin()->t - data_.begin()->t > duration_)
        data_.erase(data_.begin());
}

void StreamCurveFitting::setCurveShape(int num_pieces, double duration)
{
    n_ = num_pieces;
    curve_.setCurveShape(num_pieces);

    duration_ = duration;
}

void StreamCurveFitting::fit()
{
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

