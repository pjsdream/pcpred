#include <pcpred/detection/depth_frame_human_detector.h>

#include <algorithm>
#include <utility>

using namespace pcpred;


DepthFrameHumanDetector::DepthFrameHumanDetector()
{
    visualizer_ = 0;

    setMaxIterations(5);
    setMaxGradientDescentIterations(5);
    setMaxProjectionIterations(5);

    setGradientDescentAlpha(0.0002);
    setProjectionAlpha(0.9);
    setLengthConstraintEpsilon(0.05);

    setCapsuleDivisor(4);
}


void DepthFrameHumanDetector::loadHumanShapeFromFile(const char* filename)
{
    human_shape_.loadHumanShapeFromFile(filename);
}


Eigen::MatrixXd DepthFrameHumanDetector::medianFiltering(const Eigen::MatrixXd& depth_frame)
{
    Eigen::MatrixXd result = depth_frame;

    const int width = depth_frame.cols();
    const int height = depth_frame.rows();

    int neighbor[9][2] = {
        {-1, -1}, {-1,  0}, {-1,  1},
        { 0, -1}, { 0,  0}, { 0,  1},
        { 1, -1}, { 1,  0}, { 1,  1}
    };

    for (int i=0; i<height; i++)
    {
        for (int j=0; j<width; j++)
        {
            std::vector<double> values;
            for (int k=0; k<8; k++)
            {
                const int x = i + neighbor[k][0];
                const int y = j + neighbor[k][1];

                if (0<=x && x<height && 0<=y && y<width)
                    values.push_back(depth_frame(x, y));
            }

            std::sort(values.begin(), values.end());
            const int size = values.size();
            result(i, j) = values[size/2];
        }
    }

    return result;
}

std::vector<Eigen::Vector3d> DepthFrameHumanDetector::get3DPointCloudFromDepthFrame(const Eigen::MatrixXd& depth_frame, const Eigen::Matrix4d& intrinsic, const Eigen::Matrix4d& extrinsic)
{
    std::vector<Eigen::Vector3d> points;

    const int width = depth_frame.cols();
    const int height = depth_frame.rows();

    int cols = 0;
    for (int i=0; i<height; i++)
    {
        for (int j=0; j<width; j++)
        {
            if (depth_frame(i, j) != background_depth_)
                cols++;
        }
    }

    Eigen::Matrix4Xd B(4, cols);
    int col = 0;
    for (int i=0; i<height * width; i++)
    {
        if (depth_frame(i % height, i / height) != background_depth_)
        {
            B(0, col) = i / height + 1;
            B(1, col) = i % height + 1;
            B(2, col) = depth_frame(i % height, i / height) / (float)((1<<16) - 1);
            B(3, col) = 1.f;
            col++;
        }
    }

    // perspective
    Eigen::MatrixXd CP = intrinsic.colPivHouseholderQr().solve(B);
    for (int i=0; i<cols; i++)
    {
        for (int j=0; j<4; j++)
            CP(j,i) /= CP(3,i);
    }

    Eigen::MatrixXd GP = extrinsic.colPivHouseholderQr().solve(CP);

    // rotate
    for (int i=0; i<cols; i++)
        points.push_back( Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d(1., 0., 0.)) * GP.col(i).block(0, 0, 3, 1) );

    return points;
}

void DepthFrameHumanDetector::observeDepthFrame(Eigen::MatrixXd depth_frame, const Eigen::Matrix4d& intrinsic, const Eigen::Matrix4d& extrinsic)
{
    depth_frame = medianFiltering(depth_frame);


    const int width = depth_frame.cols();
    const int height = depth_frame.rows();

    const std::vector<Eigen::Vector3d> points = get3DPointCloudFromDepthFrame(depth_frame, intrinsic, extrinsic);
    const int num_points = points.size();

    const int num_joints = human_shape_.numJoints();
    const int num_capsules = human_shape_.numCapsules();
    std::vector<Eigen::Vector3d> joints;
    std::vector<double> joint_radius;
    std::vector<std::pair<int, int> > capsules;

    for (int i=0; i<num_joints; i++)
    {
        joints.push_back( human_shape_.jointPosition(i) );
        joint_radius.push_back( human_shape_.jointRadius(i) );
    }

    for (int i=0; i<num_capsules; i++)
        capsules.push_back( human_shape_.capsuleEndpointJointIndices(i) );

    std::vector<Eigen::Vector3d> joint_displacements(num_joints);


    // constraint lookup table initialization
    for (int i=0; i<height; i++)
    {
        for (int j=0; j<width; j++)
        {
        }
    }


    int iteration = 0;
    while (iteration < max_iterations_)
    {
        // point-capsule distance minimization
        int gradient_descent_iteration = 0;
        while (gradient_descent_iteration < max_gradient_descent_iterations_)
        {
            for (int i=0; i<num_joints; i++)
                joint_displacements[i] = Eigen::Vector3d(0., 0., 0.);

            for (int i=0; i<num_points; i++)
            {
                const Eigen::Vector3d point = points[i];

                double closest_capsule_distance = -1.0;
                int closest_capsule_joint_indices[2];
                Eigen::Vector3d closest_capsule_joint_displacements[2];

                for (int j=0; j<num_capsules; j++)
                {
                    double capsule_distance;
                    int joint_indices[2] = {0, 0};
                    Eigen::Vector3d joint_displacements[2] =
                    {
                        Eigen::Vector3d(0., 0., 0.),
                        Eigen::Vector3d(0., 0., 0.),
                    };

                    int i0 = capsules[j].first;
                    int i1 = capsules[j].second;
                    Eigen::Vector3d p0 = joints[i0];
                    Eigen::Vector3d p1 = joints[i1];
                    double r0 = joint_radius[i0];
                    double r1 = joint_radius[i1];

                    if (r0 > r1)
                    {
                        std::swap(i0, i1);
                        std::swap(p0, p1);
                        std::swap(r0, r1);
                    }

                    const double d = (p1 - p0).norm();
                    const double rd = r1 - r0;

                    if (d <= rd)
                    {
                        capsule_distance = (p1 - point).norm() - r1;
                        joint_indices[0] = i1;
                        joint_displacements[0] = (point - p1).normalized() * capsule_distance;
                    }

                    else
                    {
                        const double sin_alpha = rd / d;
                        const double cos_alpha = std::sqrt(1. - sin_alpha * sin_alpha);
                        const double tan_alpha = sin_alpha / cos_alpha;
                        const Eigen::Vector3d cross = (p1 - p0).cross(point - p0);
                        const double dot = (p1 - p0).dot(point - p0);

                        const double t = ( dot + cross.norm() * tan_alpha ) / (p1 - p0).squaredNorm();

                        if (t <= 0.0)
                        {
                            capsule_distance = (p0 - point).norm() - r0;
                            joint_indices[0] = i0;
                            joint_displacements[0] = (point - p0).normalized() * capsule_distance;
                        }

                        else if (t >= 1.0)
                        {
                            capsule_distance = (p1 - point).norm() - r1;
                            joint_indices[0] = i1;
                            joint_displacements[0] = (point - p1).normalized() * capsule_distance;
                        }

                        else
                        {
                            capsule_distance = (cross.norm() / d / cos_alpha) - ((1. - t) * r0 + t * r1);
                            const Eigen::Vector3d direction = (point - (1. - t) * p0 - t * p1).normalized() * capsule_distance;
                            joint_indices[0] = i0;
                            joint_displacements[0] = direction;
                            joint_indices[1] = i1;
                            joint_displacements[1] = direction;
                        }
                    }

                    capsule_distance = std::abs(capsule_distance);
                    if (j == 0 || closest_capsule_distance > capsule_distance)
                    {
                        closest_capsule_distance = capsule_distance;
                        closest_capsule_joint_indices[0] = joint_indices[0];
                        closest_capsule_joint_indices[1] = joint_indices[1];
                        closest_capsule_joint_displacements[0] = joint_displacements[0];
                        closest_capsule_joint_displacements[1] = joint_displacements[1];
                    }
                }

                joint_displacements[ closest_capsule_joint_indices[0] ] += closest_capsule_joint_displacements[0];
                joint_displacements[ closest_capsule_joint_indices[1] ] += closest_capsule_joint_displacements[1];
            }

            for (int i=0; i<num_joints; i++)
                joints[i] += gradient_descent_alpha_ * joint_displacements[i];

            gradient_descent_iteration++;
        }

        // cyclical projection algorithm to constrained domain
        int projection_iteration = 0;
        while (projection_iteration < max_projection_iterations_)
        {
            // length constraint

            // silhouette constraint

            // z-surface constraint

            projection_iteration++;
        }

        iteration++;
    }

    human_shape_.setJointPositions(joints);
}


void DepthFrameHumanDetector::setVisualizerTopic(const char* topic)
{
    if (visualizer_ != 0)
        delete visualizer_;

    visualizer_ = new MarkerArrayVisualizer(topic);
}

void DepthFrameHumanDetector::visualizeHuman()
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
            const double t = (double)j / capsule_divisor_;

            centers.push_back( (1.-t) * capsule_centers[0] + t * capsule_centers[1] );
            radius.push_back( (1.-t) * capsule_radius[0] + t * capsule_radius[1] );
        }
    }

    visualizer_->drawSpheres("human", centers, radius);
}
