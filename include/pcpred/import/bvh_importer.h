#ifndef BVH_IMPORTER_H
#define BVH_IMPORTER_H


#include <vector>
#include <string>
#include <map>
#include <sstream>
#include <utility>

#include <Eigen/Dense>


namespace pcpred
{

class BvhImporter
{
private:

    static const double DEGREES_TO_RADIANS;

    enum JointChannelType
    {
        JointChannelXposition= 0,
        JointChannelYposition,
        JointChannelZposition,
        JointChannelXrotation,
        JointChannelYrotation,
        JointChannelZrotation,
    };

public:

    bool import(const char* filename);

    inline int numJoints() { return joint_names_.size(); }
    inline int numFrames() { return num_frames_; }
    inline std::string jointName(int index) { return joint_names_[index]; }
    inline double frameTime() { return frame_time_; }
    inline double rate() { return 1. / frame_time_; }

    void scale(double s);
    void rotate(double angle, const Eigen::Vector3d& axis);
    void translate(const Eigen::Vector3d& t);

    Eigen::Affine3d jointTransformation(int frame_index, int joint_index);
    Eigen::Affine3d jointTransformation(int frame_index, const std::string& joint_name);
    Eigen::Affine3d jointTransformation(double time, int joint_index);
    Eigen::Affine3d jointTransformation(double time, const std::string& joint_name);
    std::vector<Eigen::Vector3d> childrenOffsets(int joint_index);

private:

    void initialize();

    void readHierarchy(FILE*& fp);
    void readHierarchy(std::istringstream& s);
    void readMotion(FILE*& fp);
    void readMotion(std::istringstream& s);
    void readHierarchyJoint(FILE*& fp, int parent_index);
    void readHierarchyJoint(std::istringstream& s, int parent_index);

    double frame_time_;
    int num_frames_;
    std::vector<std::string> joint_names_;
    std::map<std::string, int> joint_name_to_index_map_;

    // parent id should proceed the node id
    std::vector<int> parents_;
    std::vector<std::vector<int> > children_;

    std::vector<std::vector<JointChannelType> > joint_channels_;

    std::vector<Eigen::Vector3d> offsets_;

    // transformations_[frame_index][joint_index];
    std::vector<std::vector<Eigen::Affine3d> > transformations_;
};

}


#endif // BVH_IMPORTER_H
