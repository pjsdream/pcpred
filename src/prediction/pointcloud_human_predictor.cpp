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


void PointcloudHumanPredictor::setObservationTimestep(double timestep)
{
    predictor_->setObservationTimestep(timestep);
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

    is_arm_.clear();
    shoulder_sphere_index_.clear();
    length_limit_.clear();
    int shoulder_index[2][2];
    double lengths[2][2];

    sphere_radius_.clear();
    for (int i=0; i<human_shape_.numJoints(); i++)
    {
        sphere_radius_.push_back(human_shape_.jointRadius(i));

        if (human_shape_.jointName(i) == "LS")
        {
            shoulder_index[0][0] = i;
        }
        else if (human_shape_.jointName(i) == "RS")
        {
            shoulder_index[1][0] = i;
        }

        if (human_shape_.jointName(i) == "LE")
        {
            shoulder_index[0][1] = i;
            lengths[0][0] = (human_shape_.jointPosition(i) - human_shape_.jointPosition(shoulder_index[0][0])).norm();
            is_arm_.push_back(true);
            shoulder_sphere_index_.push_back(shoulder_index[0][0]);
            length_limit_.push_back(lengths[0][0]);
        }
        else if (human_shape_.jointName(i) == "LW")
        {
            lengths[0][1] = (human_shape_.jointPosition(i) - human_shape_.jointPosition(shoulder_index[0][1])).norm();
            is_arm_.push_back(true);
            shoulder_sphere_index_.push_back(shoulder_index[0][0]);
            length_limit_.push_back(lengths[0][0] + lengths[0][1]);
        }
        else if (human_shape_.jointName(i) == "RE")
        {
            shoulder_index[1][1] = i;
            lengths[1][0] = (human_shape_.jointPosition(i) - human_shape_.jointPosition(shoulder_index[1][0])).norm();
            is_arm_.push_back(true);
            shoulder_sphere_index_.push_back(shoulder_index[1][0]);
            length_limit_.push_back(lengths[1][0]);
        }
        else if (human_shape_.jointName(i) == "RW")
        {
            lengths[1][1] = (human_shape_.jointPosition(i) - human_shape_.jointPosition(shoulder_index[1][1])).norm();
            is_arm_.push_back(true);
            shoulder_sphere_index_.push_back(shoulder_index[1][0]);
            length_limit_.push_back(lengths[1][0] + lengths[1][1]);
        }
        else
        {
            is_arm_.push_back(false);
            shoulder_sphere_index_.push_back(0);
            length_limit_.push_back(0.0);
        }
    }

    for (int i=0; i<human_shape_.numCapsules(); i++)
    {
        std::string joint_names[2];
        Eigen::Vector3d capsule_centers[2];
        double capsule_radius[2];

        human_shape_.getCapsule(i, joint_names, capsule_centers, capsule_radius);

        for (int j=1; j<=capsule_divisor_; j++)
        {
            const double t = (double)j / (capsule_divisor_ + 1);

            sphere_radius_.push_back( (1.-t) * capsule_radius[0] + t * capsule_radius[1] );

            if (joint_names[1] == "LE")
            {
                is_arm_.push_back(true);
                shoulder_sphere_index_.push_back(shoulder_index[0][0]);
                length_limit_.push_back(lengths[0][0] * t);
            }
            else if (joint_names[1] == "LW")
            {
                is_arm_.push_back(true);
                shoulder_sphere_index_.push_back(shoulder_index[0][0]);
                length_limit_.push_back(lengths[0][0] + lengths[0][1] * t);
            }
            else if (joint_names[1] == "RE")
            {
                is_arm_.push_back(true);
                shoulder_sphere_index_.push_back(shoulder_index[1][0]);
                length_limit_.push_back(lengths[1][0] * t);
            }
            else if (joint_names[1] == "RW")
            {
                is_arm_.push_back(true);
                shoulder_sphere_index_.push_back(shoulder_index[1][0]);
                length_limit_.push_back(lengths[1][0] + lengths[1][1] * t);
            }
            else
            {
                is_arm_.push_back(false);
                shoulder_sphere_index_.push_back(0);
                length_limit_.push_back(0.0);
            }
        }
    }

    if (predictor_ != 0)
        delete predictor_;

    predictor_ = new PointsPredictor(numSpheres());

    // predictor initial parameter setup
    setObservationTimestep(0.1);
    setSensorDiagonalCovariance(0.001 * 0.001);
    setCollisionProbability(0.95);
    setAccelerationInferenceWindowSize(5);
}


void PointcloudHumanPredictor::rotate(double angle, const Eigen::Vector3d& axis)
{
    human_shape_.rotate(angle, axis);
}

void PointcloudHumanPredictor::translate(const Eigen::Vector3d& t)
{
    human_shape_.translate(t);
}


void PointcloudHumanPredictor::predict(double time_difference, int sphere_index, Eigen::Vector3d& mu, Eigen::Matrix3d& sigma, double& radius)
{
    predictor_->predict(time_difference, sphere_index, mu, sigma);

    if (is_arm_[sphere_index])
    {
        const double l = length_limit_[sphere_index];
        const int shoulder_sphere_index = shoulder_sphere_index_[sphere_index];
        Eigen::Vector3d s_mu;
        Eigen::Matrix3d s_sigma;

        predictor_->predict(time_difference, shoulder_sphere_index, s_mu, s_sigma);
        const double lt = (s_mu - mu).norm();

        if (lt > l)
            mu = s_mu + (mu - s_mu) / lt * l;
    }

    radius = sphere_radius_[sphere_index];
}

void PointcloudHumanPredictor::predict(double time_difference, std::vector<Eigen::Vector3d>& mu, std::vector<Eigen::Matrix3d>& sigma, std::vector<double>& radius)
{
    const int size = predictor_->numSpheres();
    mu.resize(size);
    sigma.resize(size);
    radius.resize(size);

    for (int i=0; i<size; i++)
        predict(time_difference, i, mu[i], sigma[i], radius[i]);
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

void PointcloudHumanPredictor::getPredictedEllipsoids(double time_difference, std::vector<Eigen::Vector3d>& c, std::vector<Eigen::Matrix3d>& A)
{
    std::vector<Eigen::Matrix3d> sigma;

    std::vector<double> radius;

    predict(time_difference, c, sigma, radius);

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

void PointcloudHumanPredictor::visualizePrediction(double future_time)
{
    std::vector<Eigen::Vector3d> c;  // centers of ellipsoids
    std::vector<Eigen::Matrix3d> A;  // matrixs that define ellipsoid axes

    getPredictedEllipsoids(future_time, c, A);

    char buffer[128];
    sprintf(buffer, "prediction_frame_%.3lf", future_time);

    visualizer_->drawEllipsoids(buffer, c, A);
}
