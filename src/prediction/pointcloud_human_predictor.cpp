#include <pcpred/prediction/pointcloud_human_predictor.h>

using namespace pcpred;


PointcloudHumanPredictor::PointcloudHumanPredictor()
{
    predictor_ = 0;
    visualizer_ = 0;

    // set up default parameters
    // parameters for human shape length constraint optimization are set up by its constructor
    setMaximumIterations(5);
    setGradientDescentMaximumIterations(5);
    setGradientDescentAlpha(0.1);

    setCapsuleDivisor(4);
}

void PointcloudHumanPredictor::setHumanShapeLengthConstraintEpsilon(double epsilon)
{
    human_shape_.setLengthConstraintEpsilon(epsilon);
}


int PointcloudHumanPredictor::numSpheres()
{
    return human_shape_.numJoints() + human_shape_.numCapsules() * capsule_divisor_;
}

void PointcloudHumanPredictor::loadHumanShapeFromFile(const char* filename)
{
    human_shape_.loadHumanShapeFromFile(filename);

    if (predictor_ != 0)
        delete predictor_;

    predictor_ = new PointsPredictor(numSpheres());
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

    // TODO: observe spheres from human shape
    // predictor_->observe();
}

void PointcloudHumanPredictor::optimizeHumanShape(const Eigen::Vector3d& camera_position, const std::vector<Eigen::Vector3d>& pointcloud)
{
    const int num_joints = human_shape_.numJoints();

    std::vector<int> corresponding_capsules(pointcloud.size());
    std::vector<Eigen::Vector3d> joint_displacements(num_joints);

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
                joint_displacements[i] = Eigen::Vector3d(0., 0., 0.);

            for (int i=0; i<pointcloud.size(); i++)
            {
                human_shape_.bestPullingClosestCapsule(pointcloud[i], corresponding_capsules[i], result_joint_indices, result_joint_displacements);
                joint_displacements[ result_joint_indices[0] ] += result_joint_displacements[0];
                joint_displacements[ result_joint_indices[1] ] += result_joint_displacements[1];
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
