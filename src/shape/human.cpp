#include <pcpred/shape/human.h>

#include <algorithm>

#include <stdlib.h>

#include <stdio.h>

using namespace pcpred;


Human::Human()
{
    // initialize optimization parameters by default values
    setIterativeProjectionMaximumIteration(5);
    setIterativeProjectionAlpha(0.9);
    setIterativeProjectionNumberOfSamples(100);
    setLengthConstraintEpsilon(0.1);  // 10% allowed length change

    srand(1234);
}

void Human::getCapsule(int capsule_index, Eigen::Vector3d centers[2], double radius[2])
{
    const int joint_index0 = capsules_[capsule_index].joint_ids[0];
    const int joint_index1 = capsules_[capsule_index].joint_ids[1];

    centers[0] = joints_[joint_index0].position;
    centers[1] = joints_[joint_index1].position;
    radius[0] = joints_[joint_index0].radius;
    radius[1] = joints_[joint_index1].radius;
}

void Human::loadHumanShapeFromFile(const char* filename)
{
    joint_name_to_index_map_.clear();
    joints_.clear();
    capsules_.clear();

    FILE* fp = fopen(filename, "r");

    char name1[128];
    char name2[128];

    int num_joints;
    fscanf(fp, "%d", &num_joints);
    for (int i=0; i<num_joints; i++)
    {
        double x, y, z, r;
        fscanf(fp, "%s%lf%lf%lf%lf", name1, &x, &y, &z, &r);
        addJoint(name1, Eigen::Vector3d(x, y, z), r);
    }

    int num_capsules;
    fscanf(fp, "%d", &num_capsules);
    for (int i=0; i<num_capsules; i++)
    {
        fscanf(fp, "%s%s", name1, name2);
        addCapsule(name1, name2);
    }

    fclose(fp);
}

void Human::addJoint(const char* joint_name, const Eigen::Vector3d& center, double radius)
{
    joint_name_to_index_map_[joint_name] = joints_.size();

    HumanJoint joint;
    joint.position = center;
    joint.radius = radius;
    joints_.push_back(joint);
}

void Human::addCapsule(const char* joint_name1, const char* joint_name2)
{
    HumanCapsule capsule;
    capsule.joint_ids[0] = joint_name_to_index_map_[joint_name1];
    capsule.joint_ids[1] = joint_name_to_index_map_[joint_name2];
    capsule.initial_length = (joints_[capsule.joint_ids[0]].position - joints_[capsule.joint_ids[1]].position).norm();
    capsules_.push_back(capsule);
}

void Human::jointPositionMove(int joint_index, const Eigen::Vector3d& displacement)
{
    joints_[joint_index].position += displacement;
}

void Human::setJointPositions(const std::vector<Eigen::Vector3d>& joint_positions)
{
    for (int i=0; i<joint_positions.size(); i++)
        joints_[i].position = joint_positions[i];
}

int Human::closestCapsuleIndex(const Eigen::Vector3d& point)
{
    int result = -1;
    double minimum_capsule_distance = -1;

    for (int i=0; i<capsules_.size(); i++)
    {
        double capsule_distance = 0.0;

        const int i0 = capsules_[i].joint_ids[0];
        const int i1 = capsules_[i].joint_ids[1];
        Eigen::Vector3d p0 = joints_[i0].position;
        Eigen::Vector3d p1 = joints_[i1].position;
        double r0 = joints_[i0].radius;
        double r1 = joints_[i1].radius;

        if (r0 > r1)
        {
            std::swap(p0, p1);
            std::swap(r0, r1);
        }

        const double d = (p1 - p0).norm();
        const double rd = r1 - r0;

        if (d <= rd)
            capsule_distance = (p1 - point).norm() - r1;

        else
        {
            const double sin_alpha = rd / d;
            const double cos_alpha = std::sqrt(1. - sin_alpha * sin_alpha);
            const double tan_alpha = sin_alpha / cos_alpha;
            const Eigen::Vector3d cross = (p1 - p0).cross(point - p0);
            const double dot = (p1 - p0).dot(point - p0);

            const double t = ( dot + cross.norm() * tan_alpha ) / (p1 - p0).squaredNorm();

            if (t <= 0.0)
                capsule_distance = (p0 - point).norm() - r0;

            else if (t >= 1.0)
                capsule_distance = (p1 - point).norm() - r1;

            else
                capsule_distance = (cross.norm() / d / cos_alpha) - ((1. - t) * r0 + t * r1);
        }

        capsule_distance = std::abs(capsule_distance);


        if (i==0 || minimum_capsule_distance > capsule_distance)
        {
            minimum_capsule_distance = capsule_distance;
            result = i;
        }
    }

    return result;
}

void Human::bestPullingClosestCapsule(const Eigen::Vector3d& point, int capsule_index,
                                      int joint_indices[2], Eigen::Vector3d joint_displacements[2])
{
    joint_indices[0] = 0;
    joint_indices[1] = 0;
    joint_displacements[0] = Eigen::Vector3d(0., 0., 0.);
    joint_displacements[1] = Eigen::Vector3d(0., 0., 0.);

    int i0 = capsules_[capsule_index].joint_ids[0];
    int i1 = capsules_[capsule_index].joint_ids[1];
    Eigen::Vector3d p0 = joints_[i0].position;
    Eigen::Vector3d p1 = joints_[i1].position;
    double r0 = joints_[i0].radius;
    double r1 = joints_[i1].radius;

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
        const double capsule_distance = (p1 - point).norm() - r1;
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
            const double capsule_distance = (p0 - point).norm() - r0;
            joint_indices[0] = i0;
            joint_displacements[0] = (point - p0).normalized() * capsule_distance;
            joint_indices[1] = i1;
            joint_displacements[1] = (point - p0).normalized() * capsule_distance;
        }

        else if (t >= 1.0)
        {
            const double capsule_distance = (p1 - point).norm() - r1;
            joint_indices[0] = i1;
            joint_displacements[0] = (point - p1).normalized() * capsule_distance;
            joint_indices[1] = i1;
            joint_displacements[1] = (point - p1).normalized() * capsule_distance;
        }

        else
        {
            const double capsule_distance = (cross.norm() / d / cos_alpha) - ((1. - t) * r0 + t * r1);
            const Eigen::Vector3d direction = (point - (1. - t) * p0 - t * p1).normalized() * capsule_distance;
            joint_indices[0] = i0;
            joint_displacements[0] = direction;
            joint_indices[1] = i1;
            joint_displacements[1] = direction;
        }
    }
}

void Human::projectToConstrainedDomain(const Eigen::Vector3d& camera_position, const std::vector<Eigen::Vector3d>& pointcloud)
{
    const int num_points = pointcloud.size();
    const int num_capsules = capsules_.size();

    int iteration = 0;
    bool is_converged = false;

    while (iteration < iterative_projection_max_iteration_ && !is_converged)
    {
        is_converged = true;

        // length constraints
        for (int i=0; i<num_capsules; i++)
        {
            const int i0 = capsules_[i].joint_ids[0];
            const int i1 = capsules_[i].joint_ids[1];

            Eigen::Vector3d& p0 = joints_[i0].position;
            Eigen::Vector3d& p1 = joints_[i1].position;

            const double initial_length = capsules_[i].initial_length;
            const double minimum_length = (1. - length_constraint_epsilon_) * initial_length;
            const double maximum_length = (1. + length_constraint_epsilon_) * initial_length;
            const double d = (p1 - p0).norm();

            if (d < minimum_length)
            {
                is_converged = false;

                const double l = (minimum_length - d) / 2.0;
                p0 -= iterative_projection_alpha_ * (p1 - p0).normalized() * l;
                p1 += iterative_projection_alpha_ * (p1 - p0).normalized() * l;
            }

            else if (d > maximum_length)
            {
                is_converged = false;

                const double l = (d - maximum_length) / 2.0;
                p0 += iterative_projection_alpha_ * (p1 - p0).normalized() * l;
                p1 -= iterative_projection_alpha_ * (p1 - p0).normalized() * l;
            }
        }

        // z-surface constraints
        for (int i=0; i<iterative_projection_num_samples_; i++)
        {
            // ray casting to spheres
            const int capsule_divisor = 4;
            const double threshold = 0.1;

            const Eigen::Vector3d p = pointcloud[ rand() % num_points ];
            const double measurement = (p - camera_position).norm();
            const Eigen::Vector3d v = (p - camera_position) / measurement;

            for (int j=0; j<num_capsules; j++)
            {
                const int i0 = capsules_[j].joint_ids[0];
                const int i1 = capsules_[j].joint_ids[1];
                const double r0 = joints_[i0].radius;
                const double r1 = joints_[i1].radius;

                for (int k=0; k<=capsule_divisor; k++)
                {
                    Eigen::Vector3d& p0 = joints_[i0].position;
                    Eigen::Vector3d& p1 = joints_[i1].position;

                    const double t = (double)k / capsule_divisor;

                    const Eigen::Vector3d c = (1. - t) * p0 + t * p1;
                    const double r = (1. - t) * r0 + t * r1;

                    const double s = v.dot(c - camera_position);
                    const double d = (c - camera_position).norm();
                    const double a = std::sqrt(d*d - s*s);
                    if (a <= r)
                    {
                        const double m = s - std::sqrt(r*r - a*a);
                        if (m < measurement - 0.05 && measurement - m <= threshold)
                        {
                            const Eigen::Vector3d displacement = (measurement - m - 0.05) * v;
                            p0 += 0.7 * iterative_projection_alpha_ * (1. - t) * displacement;
                            p1 += 0.7 * iterative_projection_alpha_ * t * displacement;
                        }
                    }
                }
            }

            // ray casting to capsule
            /*
            const Eigen::Vector3d point = pointcloud[ rand() % num_points ];
            const double measurement = (point - camera_position).norm();
            const Eigen::Vector3d direction = (point - camera_position) / measurement;

            for (int j=0; j<num_capsules; j++)
            {
                const double ray_distance = measurement - 0.0001; // rayCastDistanceToCapsule(j);

                if (ray_distance < measurement)
                {
                    joints_[ capsules_[j].joint_ids[0] ].position += iterative_projection_alpha_ * (measurement - ray_distance) * direction;
                    joints_[ capsules_[j].joint_ids[1] ].position += iterative_projection_alpha_ * (measurement - ray_distance) * direction;
                }
            }
            */
        }

        iteration++;
    }
}

double Human::rayCastDistanceToCapsule(int capsule_index)
{
    return 100.0;
}
