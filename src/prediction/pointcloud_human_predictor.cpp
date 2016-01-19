#include <pcpred/prediction/pointcloud_human_predictor.h>

#include <pcpred/util/chi_square.h>

#include <omp.h>

using namespace pcpred;


PointcloudHumanPredictor::PointcloudHumanPredictor()
{
    predictor_ = 0;
    visualizer_ = 0;

    // approximate a capsule with 4 spheres
    capsule_divisor_ = 4;

    // set up default parameters
    // parameters for human shape length constraint optimization are set up by its constructor
    setMaximumIterations(5);
    setGradientDescentMaximumIterations(5);
    setGradientDescentAlpha(0.1);
}

void PointcloudHumanPredictor::setHumanShapeLengthConstraintEpsilon(double epsilon)
{
    human_shape_.setLengthConstraintEpsilon(epsilon);
}


void PointcloudHumanPredictor::setTimestep(double timestep)
{
    predictor_->setTimestep(timestep);
}

void PointcloudHumanPredictor::setSensorDiagonalCovariance(double v)
{
    predictor_->setSensorDiagonalCovariance(v);
}

void PointcloudHumanPredictor::setCollisionProbability(double p)
{
    collision_probability_ = p;
}

void PointcloudHumanPredictor::setAccelerationInferenceWindowSize(double window_size)
{
    predictor_->setAccelerationInferenceWindowSize(window_size);
}


int PointcloudHumanPredictor::numSpheres()
{
    return human_shape_.numJoints() + human_shape_.numCapsules() * capsule_divisor_;
}

void PointcloudHumanPredictor::loadHumanShapeFromFile(const char* filename)
{
    human_shape_.loadHumanShapeFromFile(filename);

    sphere_radius_.clear();
    for (int i=0; i<human_shape_.numJoints(); i++)
        sphere_radius_.push_back(human_shape_.jointRadius(i));

    for (int i=0; i<human_shape_.numCapsules(); i++)
    {
        Eigen::Vector3d capsule_centers[2];
        double capsule_radius[2];

        human_shape_.getCapsule(i, capsule_centers, capsule_radius);

        for (int j=1; j<=capsule_divisor_; j++)
        {
            const double t = (double)j / (capsule_divisor_ + 1);

            sphere_radius_.push_back( (1.-t) * capsule_radius[0] + t * capsule_radius[1] );
        }
    }

    if (predictor_ != 0)
        delete predictor_;

    predictor_ = new PointsPredictor(numSpheres());

    // predictor initial parameter setup
    setTimestep(0.1);
    setSensorDiagonalCovariance(0.001 * 0.001);
    setCollisionProbability(0.95);
    setAccelerationInferenceWindowSize(5);
}


void PointcloudHumanPredictor::predict(int frame_count)
{
    predictor_->predict(frame_count);
}

void PointcloudHumanPredictor::getPredictionResult(int frame_number, int sphere_index, Eigen::Vector3d& mu, Eigen::Matrix3d& sigma)
{
    predictor_->getPredictionResult(frame_number, sphere_index, mu, sigma);
}

void PointcloudHumanPredictor::getPredictionResults(int frame_number, std::vector<Eigen::Vector3d>& mu, std::vector<Eigen::Matrix3d>& sigma)
{
    predictor_->getPredictionResults(frame_number, mu, sigma);
}


void PointcloudHumanPredictor::observe(const Eigen::Vector3d& camera_position, const Pointcloud& pointcloud)
{
    std::vector<Eigen::Vector3d> points;
    for (int i=0; i<pointcloud.size(); i++)
        points.push_back(pointcloud.point(i));

    observe(camera_position, points);
}

void PointcloudHumanPredictor::observe(const Eigen::Vector3d& camera_position, const std::vector<Eigen::Vector3d>& pointcloud)
{
    optimizeHumanShape(camera_position, pointcloud);

    // observe spheres from human shape
    std::vector<Eigen::Vector3d> centers;
    for (int i=0; i<human_shape_.numJoints(); i++)
        centers.push_back(human_shape_.jointPosition(i));

    for (int i=0; i<human_shape_.numCapsules(); i++)
    {
        Eigen::Vector3d capsule_centers[2];
        double capsule_radius[2];

        human_shape_.getCapsule(i, capsule_centers, capsule_radius);

        for (int j=1; j<=capsule_divisor_; j++)
        {
            const double t = (double)j / (capsule_divisor_ + 1);

            centers.push_back( (1.-t) * capsule_centers[0] + t * capsule_centers[1] );
        }
    }

    predictor_->observe(centers);
}

void PointcloudHumanPredictor::optimizeHumanShape(const Eigen::Vector3d& camera_position, const std::vector<Eigen::Vector3d>& pointcloud)
{
    const double sigma = 1000.0;
    const double sigma_square = sigma * sigma;


    const int num_joints = human_shape_.numJoints();

    std::vector<int> corresponding_capsules(pointcloud.size());
    std::vector<Eigen::Vector3d> joint_displacements(num_joints);

    std::vector<Eigen::Vector3d> original_positions(num_joints);
    for (int i=0; i<num_joints; i++)
        original_positions[i] = human_shape_.jointPosition(i);

    int iteration = 0;
    while (iteration < max_iteration_)
    {
        // solve for c
        for (int i=0; i<pointcloud.size(); i++)
            corresponding_capsules[i] = human_shape_.closestCapsuleIndex(pointcloud[i]);

        // solve for x using gradient descent
        int result_joint_indices[2];
        Eigen::Vector3d result_joint_displacements[2];

        for (int gradient_iteration = 0; gradient_iteration < gradient_descent_max_iteration_; gradient_iteration++)
        {
            for (int i=0; i<num_joints; i++)
                joint_displacements[i] = (original_positions[i] - human_shape_.jointPosition(i)) / sigma_square;

            for (int i=0; i<pointcloud.size(); i++)
            {
                human_shape_.bestPullingClosestCapsule(pointcloud[i], corresponding_capsules[i], result_joint_indices, result_joint_displacements);
                {
                    joint_displacements[ result_joint_indices[0] ] += result_joint_displacements[0];
                    joint_displacements[ result_joint_indices[1] ] += result_joint_displacements[1];
                }
            }

            // gradient descent step
            for (int i=0; i<num_joints; i++)
                human_shape_.jointPositionMove(i, joint_displacements[i] * gradient_descent_alpha_);
        }

        // length constraint adjustment
        human_shape_.projectToConstrainedDomain(camera_position, pointcloud);

        iteration++;
    }
}

void PointcloudHumanPredictor::getPredictedEllipsoids(int frame_number, std::vector<Eigen::Vector3d>& c, std::vector<Eigen::Matrix3d>& A)
{
    std::vector<Eigen::Matrix3d> sigma;

    predictor_->getPredictionResults(frame_number, c, sigma);

    A.resize(sigma.size());
    for (int i=0; i<sigma.size(); i++)
    {
        // svd decompose of gaussian distribution covariance matrix
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(sigma[i], Eigen::ComputeThinU | Eigen::ComputeThinV);

        // radius: square roots of eigenvalues of covariance
        const Eigen::VectorXd& r = svd.singularValues();
        const Eigen::Vector3d radius_vector =
                gaussianDistributionRadius3D(collision_probability_) * Eigen::Vector3d(std::sqrt(r(0)), std::sqrt(r(1)), std::sqrt(r(2)))
                + sphere_radius_[i] * Eigen::Vector3d(1., 1., 1.);
        const Eigen::Matrix3d radius_matrix = radius_vector.asDiagonal();

        // axis: eigenvectors of covariance
        Eigen::Matrix3d Q = svd.matrixU();

        // compose
        A[i] = Q * radius_matrix * Q.transpose();
    }
}


void PointcloudHumanPredictor::setVisualizerTopic(const char* topic)
{
    if (visualizer_ != 0)
        delete visualizer_;

    visualizer_ = new MarkerArrayVisualizer(topic);
}

void PointcloudHumanPredictor::visualizeHuman()
{
    std::vector<Eigen::Vector3d> centers;
    std::vector<double> radius;
    for (int i=0; i<human_shape_.numJoints(); i++)
    {
        centers.push_back(human_shape_.jointPosition(i));
        radius.push_back(human_shape_.jointRadius(i));
    }
    for (int i=0; i<human_shape_.numCapsules(); i++)
    {
        Eigen::Vector3d capsule_centers[2];
        double capsule_radius[2];

        human_shape_.getCapsule(i, capsule_centers, capsule_radius);

        for (int j=1; j<=capsule_divisor_; j++)
        {
            const double t = (double)j / (capsule_divisor_ + 1);

            centers.push_back( (1.-t) * capsule_centers[0] + t * capsule_centers[1] );
            radius.push_back( (1.-t) * capsule_radius[0] + t * capsule_radius[1] );
        }
    }

    visualizer_->drawSpheres("human", centers, radius);
}

void PointcloudHumanPredictor::visualizePredictionUpto(const int frame_count)
{
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
