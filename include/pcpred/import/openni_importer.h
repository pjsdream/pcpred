#ifndef OPENNI_IMPORTER_H
#define OPENNI_IMPORTER_H


#include <vector>
#include <sstream>

#include <Eigen/Dense>


namespace pcpred
{

class OpenniImporter
{
private:

    static void readStream(void* buffer, int bytes, char*& stream);

public:

    OpenniImporter();

    bool import(int sequence_number, int frame);

    std::vector<Eigen::Vector3d> pointcloud();

    inline Eigen::Vector3d cameraPosition() { return Eigen::Vector3d(0., 0., 0.); }

private:

    void readData(FILE*& fp);
    void readData(char* stream);
    void readCrop(FILE*& fp);
    void readCrop(std::istringstream& s);
    void convertToPointcloud();

    Eigen::MatrixXd raw_data_;

    Eigen::Matrix3d intrinsics_;

    std::vector<Eigen::Vector3d> pointcloud_;

    int last_sequence_number_;
    int background_intensity_;
    double crop_[4];
};

}


#endif // OPENNI_IMPORTER_H
