#ifndef GVV_DATA_IMPORTER_H
#define GVV_DATA_IMPORTER_H


#include <Eigen/Dense>

#include <vector>


namespace pcpred
{

class GVVDataImporter
{
public:

    bool import(int sequence, int frame, bool median_filter);

    void printFrameInfo();

    const std::vector<Eigen::Vector3d>& pointcloud();

private:

    void loadDepthFrameFromFile(const char* filename);
    void get3DPointCloudFromDepthFrame(bool median_filter);
    bool existFile(const char* filename);

    int sequence_;
    int frame_;

    // depth frame
    short dim_[2];
    int memory_layout_;
    short min_;
    short max_;
    short special_;
    Eigen::Matrix4d extrinsics_;
    Eigen::Matrix4d intrinsics_;
    int size_;
    Eigen::MatrixXd data_;

    // point cloud
    std::vector<Eigen::Vector3d> pointcloud_;
};

}


#endif // GVV_DATA_IMPORTER_H
