#include <pcpred/import/gvv_data_importer.h>

#include <cassert>

#include <stdio.h>
#include <iostream>

#include <ros/console.h>

using namespace pcpred;


void GVVDataImporter::printFrameInfo()
{
    std::cout << "Sequence " << sequence_ << " Frame " << frame_ << std::endl;

    std::cout << "Dim = [" << dim_[0] << ", " << dim_[1] << "]" << std::endl;
    std::cout << "Memory layout = " << memory_layout_ << std::endl;
    std::cout << "Min/max = [" << min_ << ", " << max_ << "]" << std::endl;
    std::cout << "Special = " << special_ << std::endl;
    std::cout << "Extrinsic = " << std::endl << extrinsics_ << std::endl;
    std::cout << "Intrinsic = " << std::endl << intrinsics_ << std::endl;
    std::cout << "Size = " << size_ << std::endl;
}

const std::vector<Eigen::Vector3d>& GVVDataImporter::pointcloud()
{
    return pointcloud_;
}


bool GVVDataImporter::import(int sequence, int frame)
{
    char filename[128];
    sprintf(filename, "../data/D%d/depth%04d.bin", sequence, frame);

    // file existance check
    FILE* fp = fopen(filename, "rb");
    if (fp == 0)
        return false;
    fclose(fp);

    loadDepthFrameFromFile(filename);
    get3DPointCloudFromDepthFrame();
}

void GVVDataImporter::loadDepthFrameFromFile(const char* filename)
{
    FILE* fp = fopen(filename, "rb");

    short magic_number;
    fread(&magic_number, sizeof(short), 1, fp);
    assert(magic_number == 28746); // Unknown file type!

    fread(dim_, sizeof(short), 2, fp);
    fread(&memory_layout_, sizeof(int), 1, fp);
    fread(&min_, sizeof(short), 1, fp);
    fread(&max_, sizeof(short), 1, fp);
    fread(&special_, sizeof(short), 1, fp);

    float matrix[16];
    fread(matrix, sizeof(float), 16, fp);
    extrinsics_ = Eigen::Map<Eigen::Matrix4f>(matrix).cast<double>(); // both matlab fread and eigen map fill in column-major order
    fread(matrix, sizeof(float), 16, fp);
    intrinsics_ = Eigen::Map<Eigen::Matrix4f>(matrix).cast<double>();

    fread(&size_, sizeof(float), 1, fp);

    unsigned short* raw_data_short_ = new unsigned short[size_ / 2];
    fread(raw_data_short_, sizeof(unsigned short), size_ / 2, fp);

    double* raw_data_ = new double[size_ / 2];
    for (int i=0; i<size_ / 2; i++)
        raw_data_[i] = raw_data_short_[i];

    assert(min_ == 0 && max_ == 0 && special_ == 0); // Only 16-Bit uncompressed depth maps are supported!

    switch (memory_layout_)
    {
    case 0:
        data_ = Eigen::Map<Eigen::MatrixXd>(raw_data_, (int)dim_[0], (int)dim_[1]).transpose().colwise().reverse();
        break;
    default:
        ROS_ERROR("GVV data importer: Memory layout %d not yet supported!", memory_layout_);
    }

    delete raw_data_short_;
    delete raw_data_;
    fclose(fp);
}

void GVVDataImporter::get3DPointCloudFromDepthFrame()
{
    pointcloud_.clear();

    const int V = data_.rows();
    const int U = data_.cols();

    // ignore background
    int cols = 0;
    for (int i=0; i<U*V; i++)
    {
        if (data_(i%V, i/V) != 65535)
            cols++;
    }
    if (cols == 0)
        return;

    Eigen::Matrix4Xd B(4, cols);
    int col = 0;
    for (int i=0; i<U*V; i++)
    {
        if (data_(i%V, i/V) != 65535)
        {
            B(0, col) = i / V + 1;
            B(1, col) = i % V + 1;
            B(2, col) = data_(i%V, i/V) / (float)((1<<16) - 1);
            B(3, col) = 1.f;
            col++;
        }
    }

    Eigen::MatrixXd CP = intrinsics_.colPivHouseholderQr().solve(B);
    for (int i=0; i<cols; i++)
    {
        for (int j=0; j<4; j++)
            CP(j,i) /= CP(3,i);
    }

    Eigen::MatrixXd GP = extrinsics_.colPivHouseholderQr().solve(CP);

    for (int i=0; i<cols; i++)
        pointcloud_.push_back(GP.col(i).block(0, 0, 3, 1));
}
