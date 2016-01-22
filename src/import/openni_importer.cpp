#include <pcpred/import/openni_importer.h>

#include <stdio.h>

using namespace pcpred;


bool OpenniImporter::import(int sequence_number, int frame)
{
    char filename[128];
    sprintf(filename, "../data/C%d/image%04d.txt", sequence_number, frame);
    FILE* fp = fopen(filename, "rb");
    if (fp == NULL)
        return false;

    float intrinsics[9];
    fread(intrinsics, sizeof(float), 9, fp);
    intrinsics_ = Eigen::Map<Eigen::Matrix<float, 3, 3> >(intrinsics).cast<double>();

    unsigned short dim[2];
    fread(dim, sizeof(unsigned short), 2, fp);

    unsigned short* data = new unsigned short[dim[0] * dim[1]];
    fread(data, sizeof(unsigned short), dim[0] * dim[1], fp);

    raw_data_.resize(dim[0], dim[1]);
    for (int i=0; i<dim[0]; i++)
    {
        for (int j=0; j<dim[1]; j++)
        {
            raw_data_(i, j) = data[i*dim[1] + j];
            if (raw_data_(i, j) == 0)
                raw_data_(i, j) = 65535;
        }
    }

    delete data;
    fclose(fp);



    const double background_intensity = 2800;

    pointcloud_.clear();

    const int V = raw_data_.rows();
    const int U = raw_data_.cols();

    // ignore background
    int cols = 0;
    for (int i=0; i<U*V; i++)
    {
        if (raw_data_(i%V, i/V) < background_intensity)
            cols++;
    }
    if (cols == 0)
        return true;

    Eigen::Matrix3Xd B(3, cols);
    int col = 0;
    for (int i=0; i<U*V; i++)
    {
        const int x = i/V;
        const int y = i%V;

        if (raw_data_(i%V, i/V) < background_intensity &&
                0.18*U <= x && x <= 0.6*U &&
                0.0*V <= y && y <= 0.8*V)
        {
            B(0, col) = i / V;
            B(1, col) = i % V;
            B(2, col) = 1.0;
            col++;
        }
    }


    Eigen::MatrixXd CP = intrinsics_.colPivHouseholderQr().solve(B);
    for (int i=0; i<cols; i++)
        CP.block(0, i, 3, 1) /= CP(2,i);

    col = 0;
    for (int i=0; i<U*V; i++)
    {
        const int x = i/V;
        const int y = i%V;

        if (raw_data_(i%V, i/V) < background_intensity &&
                0.18*U <= x && x <= 0.6*U &&
                0.0*V <= y && y <= 0.8*V)
        {
            pointcloud_.push_back(CP.block(0, col, 3, 1) * raw_data_(i%V, i/V) / 1000.0);
            col++;
        }
    }

    return true;
}

std::vector<Eigen::Vector3d> OpenniImporter::pointcloud()
{
    return pointcloud_;
}
