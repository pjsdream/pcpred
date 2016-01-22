#ifndef OPENNI_IMPORTER_H
#define OPENNI_IMPORTER_H


#include <vector>

#include <Eigen/Dense>


namespace pcpred
{

class OpenniImporter
{
public:

    bool import(int sequence_number, int frame);

    std::vector<Eigen::Vector3d> pointcloud();

    inline Eigen::Vector3d cameraPosition() { return Eigen::Vector3d(0., 0., 0.); }

private:

    Eigen::MatrixXd raw_data_;

    Eigen::Matrix3d intrinsics_;

    std::vector<Eigen::Vector3d> pointcloud_;

};

}


#endif // OPENNI_IMPORTER_H
