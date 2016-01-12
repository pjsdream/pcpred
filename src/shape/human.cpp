#include <pcpred/shape/human.h>

#include <stdio.h>

using namespace pcpred;


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
    capsules_.push_back(capsule);
}

int Human::closestCapsuleIndex(const Eigen::Vector3d& point)
{
    int result = -1;

    for (int i=0; i<capsules_.size(); i++)
    {
        // TODO: point to capsule distance evaluation

        result = i;
    }

    return result;
}

void Human::bestPullingClosestCapsule(const Eigen::Vector3d& point, int capsule_index,
                                      int joint_indices[2], Eigen::Vector3d joint_displacements[2])
{
    // TODO: point to capsule distance evaluation
}

void Human::projectToJointLengthConstraint()
{
    int iteration = 0;
    while (iteration < iterative_projection_max_iteration_)
    {
        // TODO: joint move

        iteration++;
    }
}
