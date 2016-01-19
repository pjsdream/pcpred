#include <pcpred/prediction/gvv_predictor.h>

#include <stdio.h>

using namespace pcpred;


const double GvvPredictor::motion_timestep_ = 0.033;


GvvPredictor::GvvPredictor(int sequence_number)
{
    sequence_number_ = sequence_number;

    pointcloud_visualizer_ = 0;
    time_ = -1.0;

    transformation_.setIdentity();

    // Y-up to Z-up
    rotate(M_PI / 2.0, Eigen::Vector3d(1., 0., 0.));

    // load human shape
    char filename[128];
    sprintf(filename, "../data/human/D%d.txt", sequence_number);
    predictor_.loadHumanShapeFromFile(filename);

    // initial parameters setup
    setTimestep(0.05);
    setSensorDiagonalCovariance(0.01 * 0.01);   // variance is proportional to square of sensing error
    setCollisionProbability(0.95);
    setAccelerationInferenceWindowSize(5);

    setMaximumIterations(5);
    setGradientDescentMaximumIterations(5);
    setGradientDescentAlpha(0.005);
    setHumanShapeLengthConstraintEpsilon(0.01);
    setCapsuleDivisor(4);
}


void GvvPredictor::rotate(double angle, const Eigen::Vector3d& axis)
{
    Eigen::AngleAxisd aa(angle, axis);
    transformation_.prerotate(aa);
}

void GvvPredictor::translate(const Eigen::Vector3d& t)
{
    transformation_.pretranslate(t);
}


void GvvPredictor::setTimestep(double timestep)
{
    timestep_ = timestep;
    predictor_.setTimestep(timestep);
}

void GvvPredictor::setSensorDiagonalCovariance(double v)
{
    predictor_.setSensorDiagonalCovariance(v);
}

void GvvPredictor::setAccelerationInferenceWindowSize(int window_size)
{
    predictor_.setAccelerationInferenceWindowSize(window_size);
}

void GvvPredictor::setCollisionProbability(double probability)
{
    predictor_.setCollisionProbability(probability);
}


void GvvPredictor::setMaximumIterations(int iterations)
{
    predictor_.setMaximumIterations(iterations);
}

void GvvPredictor::setGradientDescentMaximumIterations(int iterations)
{
    predictor_.setGradientDescentMaximumIterations(iterations);
}

void GvvPredictor::setGradientDescentAlpha(double alpha)
{
    predictor_.setGradientDescentAlpha(alpha);
}

void GvvPredictor::setHumanShapeLengthConstraintEpsilon(double epsilon)
{
    predictor_.setHumanShapeLengthConstraintEpsilon(epsilon);
}

void GvvPredictor::setCapsuleDivisor(int divisor)
{
    predictor_.setCapsuleDivisor(divisor);
}


void GvvPredictor::moveToNextFrame()
{
    if (time_ < 0.0)
        time_ = 0.0;
    else
        time_ += timestep_;

    const int frame = time_ / motion_timestep_;

    if (importer_.import(sequence_number_, frame, false) == false ||
        importer_.pointcloud().size() == 0)
        importer_.import(sequence_number_, last_imported_frame_, false);
    else
        last_imported_frame_ = frame;

    Pointcloud original_pointcloud = importer_.pointcloud();

    pointcloud_.clear();
    const int size = original_pointcloud.size();
    for (int i=0; i<3000; i++)
        pointcloud_.push_back( transformation_ * original_pointcloud.point( rand() % size ) );

    const Eigen::Vector3d camera_position = transformation_ * importer_.cameraPosition();

    predictor_.observe(camera_position, pointcloud_);
}


void GvvPredictor::predict(int frame_count)
{
    predictor_.predict(frame_count);
}


void GvvPredictor::getPredictedGaussianDistribution(int frame_number, std::vector<Eigen::Vector3d>& mu, std::vector<Eigen::Matrix3d>& sigma)
{
    predictor_.getPredictionResults(frame_number, mu, sigma);
}


void GvvPredictor::setVisualizerTopic(const char* topic)
{
    if (pointcloud_visualizer_ != 0)
        delete pointcloud_visualizer_;

    std::string pointcloud_topic = std::string(topic) + "_pointcloud";
    pointcloud_visualizer_ = new PointcloudVisualizer(pointcloud_topic.c_str());

    predictor_.setVisualizerTopic(topic);
}

void GvvPredictor::visualizePointcloud()
{
    pointcloud_visualizer_->drawPointcloud(pointcloud_);
}

void GvvPredictor::visualizeHuman()
{
    predictor_.visualizeHuman();
}

void GvvPredictor::visualizePredictionUpto(int frame_count)
{
    predictor_.visualizePredictionUpto(frame_count);
}