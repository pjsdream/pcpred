#include <pcpred/prediction/bvh_predictor.h>

#include <pcpred/util/chi_square.h>

#include <stdio.h>

using namespace pcpred;


BvhPredictor::BvhPredictor(const char* filename)
{
    // import bvh file
    bvh_importer_.import(filename);

    // centimeters to meters
    bvh_importer_.scale(0.01);

    // rotate from Y-up to Z-up
    bvh_importer_.rotate(M_PI / 2.0, Eigen::Vector3d(1.0, 0.0, 0.0));


    // TODO: initialize predictor with the number of spheres to be predicted
    const int num_joints = bvh_importer_.numJoints();
    const double sphere_radius = 0.05; // all spheres have radius 5cm

    // sphere at root joint
    JointBoundSphere sphere;
    sphere.joint_index = 0;
    sphere.position = Eigen::Vector3d(0., 0., 0.);
    sphere.radius = sphere_radius;
    spheres_.push_back(sphere);

    for (int i=0; i<num_joints; i++)
    {
        const std::vector<Eigen::Vector3d> offsets = bvh_importer_.childrenOffsets(i);

        for (int j=0; j<offsets.size(); j++)
        {
            const double length = offsets[j].norm();
            const int sphere_count = (int)(length / (sphere_radius * 2.0)) + 2;

            for (int k=1; k<sphere_count; k++)
            {
                const double t = (double) k / (sphere_count - 1);

                sphere.joint_index = i;
                sphere.position = t * offsets[j];
                sphere.radius = sphere_radius;
                spheres_.push_back(sphere);
            }
        }
    }

    points_predictor_ = new PointsPredictor(spheres_.size());


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
    points_predictor_->setObservationTimestep(timestep);
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
    for (int i=0; i<spheres_.size(); i++)
        points.push_back( bvh_importer_.jointTransformation(time_, spheres_[i].joint_index) * spheres_[i].position );

    points_predictor_->observe(points);
}

void BvhPredictor::getPredictedEllipsoids(double future_time, std::vector<Eigen::Vector3d>& c, std::vector<Eigen::Matrix3d>& A)
{
    std::vector<Eigen::Matrix3d> sigma;

    points_predictor_->predict(future_time, c, sigma);

    A.resize(sigma.size());
    for (int i=0; i<sigma.size(); i++)
    {
        // svd decompose of gaussian distribution covariance matrix
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(sigma[i], Eigen::ComputeThinU | Eigen::ComputeThinV);

        // radius: square roots of eigenvalues of covariance
        const Eigen::VectorXd& r = svd.singularValues();
        const Eigen::Vector3d radius_vector =
                gaussianDistributionRadius3D(collision_probability_) * Eigen::Vector3d(std::sqrt(r(0)), std::sqrt(r(1)), std::sqrt(r(2)))
                + spheres_[i].radius * Eigen::Vector3d(1., 1., 1.);
        const Eigen::Matrix3d radius_matrix = radius_vector.asDiagonal();

        // axis: eigenvectors of covariance
        Eigen::Matrix3d Q = svd.matrixU();

        // compose
        A[i] = Q * radius_matrix * Q.transpose();
    }
}

void BvhPredictor::getPredictedGaussianDistribution(double future_time, std::vector<Eigen::Vector3d>& mu, std::vector<Eigen::Matrix3d>& sigma, std::vector<double>& radius)
{
    points_predictor_->predict(future_time, mu, sigma);

    radius.resize( spheres_.size() );
    for (int i=0; i<spheres_.size(); i++)
        radius[i] = spheres_[i].radius;
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
    std::vector<double> radius;
    for (int i=0; i<spheres_.size(); i++)
    {
        centers.push_back( bvh_importer_.jointTransformation(time_, spheres_[i].joint_index) * spheres_[i].position );
        radius.push_back(spheres_[i].radius);
    }

    visualizer_->drawSpheres("human", centers, radius);
}

void BvhPredictor::visualizePrediction(double future_time)
{
    std::vector<Eigen::Vector3d> c;  // centers of ellipsoids
    std::vector<Eigen::Matrix3d> A;  // matrixs that define ellipsoid axes

    getPredictedEllipsoids(future_time, c, A);

    char buffer[128];
    sprintf(buffer, "prediction_frame_%.3lf", future_time);

    visualizer_->drawEllipsoids(buffer, c, A);
}
