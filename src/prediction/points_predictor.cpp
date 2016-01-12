#include <pcpred/prediction/points_predictor.h>

using namespace pcpred;


PointsPredictor::PointsPredictor(int num_spheres)
    : num_spheres_(num_spheres)
{
    point_predictors_.resize(num_spheres);
}

void PointsPredictor::setTimestep(double timestep)
{
    for (int i=0; i<point_predictors_.size(); i++)
        point_predictors_[i].setTimestep(timestep);
}

void PointsPredictor::setSensorDiagonalCovariance(double v)
{
    for (int i=0; i<point_predictors_.size(); i++)
        point_predictors_[i].setSensorDiagonalCovariance(v);
}

void PointsPredictor::setAccelerationInferenceWindowSize(int window_size)
{
    for (int i=0; i<point_predictors_.size(); i++)
        point_predictors_[i].setAccelerationInferenceWindowSize(window_size);
}


void PointsPredictor::observe(const std::vector<Eigen::Vector3d>& p)
{
    for (int i=0; i<point_predictors_.size(); i++)
        point_predictors_[i].observe(p[i]);
}

void PointsPredictor::predict(int frame_count)
{
    for (int i=0; i<point_predictors_.size(); i++)
        point_predictors_[i].predict(frame_count);
}

void PointsPredictor::getPredictionResult(int frame_number, int sphere_index, Eigen::Vector3d& mu, Eigen::Matrix3d& sigma)
{
    point_predictors_[sphere_index].getPredictionResult(frame_number, mu, sigma);
}

void PointsPredictor::getPredictionResults(int frame_number, std::vector<Eigen::Vector3d>& mu, std::vector<Eigen::Matrix3d>& sigma)
{
    mu.resize(point_predictors_.size());
    sigma.resize(point_predictors_.size());

    for (int i=0; i<point_predictors_.size(); i++)
        point_predictors_[i].getPredictionResult(frame_number, mu[i], sigma[i]);
}
