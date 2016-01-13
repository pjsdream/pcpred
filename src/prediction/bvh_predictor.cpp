#include <pcpred/prediction/bvh_predictor.h>

#include <stdio.h>

using namespace pcpred;


bool BvhPredictor::gaussian_distribution_radius_table_initialized_ = false;
std::map<double, double> BvhPredictor::gaussian_distribution_radius_table_;


BvhPredictor::BvhPredictor(const char* filename)
{
    // initialize chi square table
    if (gaussian_distribution_radius_table_initialized_ == false)
    {
        // from chi-squared distribution wiki page for dimension = 3
        gaussian_distribution_radius_table_initialized_ = true;
        gaussian_distribution_radius_table_[1.0 - 0.95 ] = std::sqrt( 0.35);
        gaussian_distribution_radius_table_[1.0 - 0.90 ] = std::sqrt( 0.58);
        gaussian_distribution_radius_table_[1.0 - 0.80 ] = std::sqrt( 1.01);
        gaussian_distribution_radius_table_[1.0 - 0.70 ] = std::sqrt( 1.42);
        gaussian_distribution_radius_table_[1.0 - 0.50 ] = std::sqrt( 2.37);
        gaussian_distribution_radius_table_[1.0 - 0.30 ] = std::sqrt( 3.66);
        gaussian_distribution_radius_table_[1.0 - 0.20 ] = std::sqrt( 4.64);
        gaussian_distribution_radius_table_[1.0 - 0.10 ] = std::sqrt( 6.25);
        gaussian_distribution_radius_table_[1.0 - 0.05 ] = std::sqrt( 7.82);
        gaussian_distribution_radius_table_[1.0 - 0.01 ] = std::sqrt(11.34);
        gaussian_distribution_radius_table_[1.0 - 0.001] = std::sqrt(16.27);
    }


    // import bvh file
    bvh_importer_.import(filename);

    // centimeters to meters
    bvh_importer_.scale(0.01);

    // rotate from Y-up to Z-up
    bvh_importer_.rotate(M_PI / 2.0, Eigen::Vector3d(1.0, 0.0, 0.0));


    // TODO: initialize predictor with the number of spheres to be predicted
    // Currently it create spheres of radius 10cm centered at every joint
    points_predictor_ = new PointsPredictor(bvh_importer_.numJoints());
    sphere_sizes_ = std::vector<double>(bvh_importer_.numJoints(), 0.1);


    // initial parameter setup
    time_ = -1.0;
    setTimestep(0.1);
    setSensorDiagonalCovariance(0.001 * 0.001);
    setAccelerationInferenceWindowSize(5);
    setCollisionProbability(0.95);

    // initialize visualizer with null pointer
    visualizer_ = 0;
}


void BvhPredictor::rotate(double angle, const Eigen::Vector3d& axis)
{
    bvh_importer_.rotate(angle, axis);
}

void BvhPredictor::translate(const Eigen::Vector3d& t)
{
    bvh_importer_.translate(t);
}


void BvhPredictor::setTimestep(double timestep)
{
    timestep_ = timestep;
    points_predictor_->setTimestep(timestep);
}

void BvhPredictor::setSensorDiagonalCovariance(double v)
{
    points_predictor_->setSensorDiagonalCovariance(v);
}

void BvhPredictor::setAccelerationInferenceWindowSize(int window_size)
{
    points_predictor_->setAccelerationInferenceWindowSize(window_size);
}

void BvhPredictor::setCollisionProbability(double probability)
{
    collision_probability_ = probability;
}


void BvhPredictor::moveToNextFrame()
{
    if (time_ < 0.0)
        time_ = 0.0;
    else
        time_ += timestep_;

    // TODO: observe spheres
    std::vector<Eigen::Vector3d> points;
    for (int i=0; i<bvh_importer_.numJoints(); i++)
        points.push_back( bvh_importer_.jointTransformation(time_, i).translation() );
    points_predictor_->observe(points);
}

void BvhPredictor::predict(int frame_count)
{
    if (time_ < 0.0)
        moveToNextFrame();

    points_predictor_->predict(frame_count);
}

void BvhPredictor::getPredictedEllipsoids(int frame_number, std::vector<Eigen::Vector3d>& c, std::vector<Eigen::Matrix3d>& A)
{
    std::vector<Eigen::Matrix3d> sigma;

    points_predictor_->getPredictionResults(frame_number, c, sigma);

    A.resize(sigma.size());
    for (int i=0; i<sigma.size(); i++)
    {
        // svd decompose of gaussian distribution covariance matrix
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(sigma[i], Eigen::ComputeThinU | Eigen::ComputeThinV);

        // radius: square roots of eigenvalues of covariance
        const Eigen::VectorXd& r = svd.singularValues();
        const Eigen::Vector3d radius_vector =
                gaussian_distribution_radius_table_[collision_probability_] * Eigen::Vector3d(std::sqrt(r(0)), std::sqrt(r(1)), std::sqrt(r(2)))
                + sphere_sizes_[i] * Eigen::Vector3d(1., 1., 1.);
        const Eigen::Matrix3d radius_matrix = radius_vector.asDiagonal();

        // axis: eigenvectors of covariance
        Eigen::Matrix3d Q = svd.matrixU();

        // compose
        A[i] = Q * radius_matrix * Q.transpose();
    }
}


void BvhPredictor::setVisualizerTopic(const char* topic)
{
    if (visualizer_ != 0)
        delete visualizer_;

    visualizer_ = new MarkerArrayVisualizer(topic);
}

void BvhPredictor::visualizeHuman()
{
    std::vector<Eigen::Vector3d> centers;
    for (int i=0; i<bvh_importer_.numJoints(); i++)
        centers.push_back( bvh_importer_.jointTransformation(time_, i).translation() );

    visualizer_->drawSpheres("human", centers, sphere_sizes_);
}

void BvhPredictor::visualizePredictionUpto(int frame_count)
{
    const int num_joints = bvh_importer_.numJoints();

    for (int i=0; i<frame_count; i++)
    {
        std::vector<Eigen::Vector3d> c;  // centers of ellipsoids
        std::vector<Eigen::Matrix3d> A;  // matrixs that define ellipsoid axes

        getPredictedEllipsoids(i, c, A);

        char buffer[128];
        sprintf(buffer, "prediction_frame%d", i);

        visualizer_->drawEllipsoids(buffer, c, A);
    }
}
