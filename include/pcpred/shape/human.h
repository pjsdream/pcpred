#ifndef HUMAN_H
#define HUMAN_H


#include <Eigen/Dense>

#include <string>
#include <map>
#include <vector>


namespace pcpred
{

class Human
{
private:

    struct HumanJoint
    {
        Eigen::Vector3d position;
        double radius;
    };

    struct HumanCapsule
    {
        int joint_ids[2];
        double initial_length;
    };

public:

    Human();

    inline int numJoints() { return joints_.size(); }
    inline int numCapsules() { return capsules_.size(); }
    inline Eigen::Vector3d jointPosition(int joint_index) { return joints_[joint_index].position; }
    inline double jointRadius(int joint_index) { return joints_[joint_index].radius; }

    void getCapsule(int capsule_index, Eigen::Vector3d centers[2], double radius[2]);

    inline void setIterativeProjectionMaximumIteration(int iteration) { iterative_projection_max_iteration_ = iteration; }
    inline void setIterativeProjectionAlpha(double alpha) { iterative_projection_alpha_ = alpha; }
    inline void setLengthConstraintEpsilon(double epsilon) { length_constraint_epsilon_ = epsilon; }

    void loadHumanShapeFromFile(const char* filename);

    void addJoint(const char* joint_name, const Eigen::Vector3d& center, double radius);
    void addCapsule(const char* joint_name1, const char* joint_name2);
    void jointPositionMove(int joint_index, const Eigen::Vector3d& displacement);

    int closestCapsuleIndex(const Eigen::Vector3d& point);
    void bestPullingClosestCapsule(const Eigen::Vector3d& point, int capsule_index,
                                   int joint_indices[2], Eigen::Vector3d joint_displacements[2]);
    void projectToJointLengthConstraintedDomain();

private:

    std::vector<HumanJoint> joints_;
    std::map<std::string, int> joint_name_to_index_map_;
    std::vector<HumanCapsule> capsules_;

    int iterative_projection_max_iteration_;
    double iterative_projection_alpha_;
    double length_constraint_epsilon_;
};

}


#endif // HUMAN_H
