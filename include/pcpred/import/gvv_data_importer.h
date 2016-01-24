#ifndef GVV_DATA_IMPORTER_H
#define GVV_DATA_IMPORTER_H


#include <Eigen/Dense>

#include <vector>


namespace pcpred
{

class GVVDataImporter
{
private:

    static void readStream(void* buffer, int bytes, char*& stream);

public:

    bool import(int sequence, int frame, bool median_filter);
    bool importDepthFrame(int sequence, int frame);

    void printFrameInfo();

    const std::vector<Eigen::Vector3d>& pointcloud();
    const Eigen::MatrixXd& depthFrame();
    inline const Eigen::Matrix4d& intrinsic() { return intrinsics_; }
    inline const Eigen::Matrix4d& extrinsic() { return extrinsics_; }
    Eigen::Vector3d cameraPosition();
    std::vector<Eigen::Vector3d> cameraEndpoints();

private:

    void loadDepthFrameFromFile(const char* filename);
    void loadDepthFrameFromFile(char* stream);
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
