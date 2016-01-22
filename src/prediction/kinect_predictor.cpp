#include <pcpred/prediction/kinect_predictor.h>

#include <stdio.h>

using namespace pcpred;


const double KinectPredictor::motion_timestep_ = 0.033;


KinectPredictor::KinectPredictor(int sequence_number)
{
    sequence_number_ = sequence_number;

    pointcloud_visualizer_ = 0;
    time_ = -1.0;

    transformation_.setIdentity();

    // initial transformation
    rotate(-M_PI / 2.0, Eigen::Vector3d(1.0, 0.0, 0.0));
    rotate(M_PI / 2.0, Eigen::Vector3d(0.0, 0.0, 1.0));
    translate(Eigen::Vector3d(2.0, 0.0, 0.75));

    // load human shape
    char filename[128];
    sprintf(filename, "../data/human/C%d.txt", sequence_number);
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
}


void KinectPredictor::rotate(double angle, const Eigen::Vector3d& axis)
{
    Eigen::AngleAxisd aa(angle, axis);
    transformation_.prerotate(aa);
}

void KinectPredictor::translate(const Eigen::Vector3d& t)
{
    transformation_.pretranslate(t);
}


void KinectPredictor::setTimestep(double timestep)
{
    timestep_ = timestep;
    predictor_.setObservationTimestep(timestep);
}

void KinectPredictor::setSensorDiagonalCovariance(double v)
{
    predictor_.setSensorDiagonalCovariance(v);
}

void KinectPredictor::setAccelerationInferenceWindowSize(int window_size)
{
    predictor_.setAccelerationInferenceWindowSize(window_size);
}

void KinectPredictor::setCollisionProbability(double probability)
{
    predictor_.setCollisionProbability(probability);
}


void KinectPredictor::setMaximumIterations(int iterations)
{
    predictor_.setMaximumIterations(iterations);
}

void KinectPredictor::setGradientDescentMaximumIterations(int iterations)
{
    predictor_.setGradientDescentMaximumIterations(iterations);
}

void KinectPredictor::setGradientDescentAlpha(double alpha)
{
    predictor_.setGradientDescentAlpha(alpha);
}

void KinectPredictor::setHumanShapeLengthConstraintEpsilon(double epsilon)
{
    predictor_.setHumanShapeLengthConstraintEpsilon(epsilon);
}


void KinectPredictor::moveToNextFrame()
{
    if (time_ < 0.0)
        time_ = 0.0;
    else
        time_ += timestep_;

    const int frame = time_ / motion_timestep_;

    if (importer_.import(sequence_number_, frame) == false ||
        importer_.pointcloud().size() == 0)
        importer_.import(sequence_number_, last_imported_frame_);
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


void KinectPredictor::getPredictedGaussianDistribution(double future_time, std::vector<Eigen::Vector3d>& mu, std::vector<Eigen::Matrix3d>& sigma, std::vector<double>& radius)
{
    predictor_.predict(future_time, mu, sigma, radius);
}


void KinectPredictor::setVisualizerTopic(const char* topic)
{
    if (pointcloud_visualizer_ != 0)
        delete pointcloud_visualizer_;

    std::string pointcloud_topic = std::string(topic) + "_pointcloud";
    pointcloud_visualizer_ = new PointcloudVisualizer(pointcloud_topic.c_str());

    predictor_.setVisualizerTopic(topic);
}

void KinectPredictor::visualizePointcloud()
{
    pointcloud_visualizer_->drawPointcloud(pointcloud_);
}

void KinectPredictor::visualizeHuman()
{
    predictor_.visualizeHuman();
}

void KinectPredictor::visualizePrediction(double future_time)
{
    predictor_.visualizePrediction(future_time);
}
