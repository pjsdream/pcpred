#include <pcpred/import/openni_importer.h>

#include <resource_retriever/retriever.h>

#include <stdio.h>

using namespace pcpred;



void OpenniImporter::readStream(void* buffer, int bytes, char*& stream)
{
    memcpy(buffer, stream, bytes);
    stream += bytes;
}


OpenniImporter::OpenniImporter()
{
    last_sequence_number_ = -1;
}


bool OpenniImporter::import(int sequence_number, int frame)
{
    char filename[128];
    char package_filename[128];

    sprintf(filename, "../data/C%d/image%04d.txt", sequence_number, frame);
    FILE* fp = fopen(filename, "rb");

    if (fp == NULL)
    {
        resource_retriever::Retriever retriever;
        resource_retriever::MemoryResource resource;

        sprintf(package_filename, "package://pcpred/data/C%d/image%04d.txt", sequence_number, frame);

        try
        {
            resource = retriever.get(package_filename);
        }
        catch (resource_retriever::Exception& e)
        {
            return false;
        }

        readData((char*)resource.data.get());
    }
    else
    {
        readData(fp);
    }


    if (last_sequence_number_ != sequence_number)
    {
        sprintf(filename, "../data/C%d/crop.txt", sequence_number);
        fp = fopen(filename, "r");
        if (fp == NULL)
        {
            resource_retriever::Retriever retriever;
            resource_retriever::MemoryResource resource;

            sprintf(package_filename, "package://pcpred/data/C%d/crop.txt", sequence_number);

            try
            {
                resource = retriever.get(package_filename);
            }
            catch (resource_retriever::Exception& e)
            {
                return false;
            }

            std::istringstream s((char*)resource.data.get());
            readCrop(s);
        }
        else
        {
            readCrop(fp);
        }

        last_sequence_number_ = sequence_number;
    }

    convertToPointcloud();
    return true;
}

void OpenniImporter::readData(FILE*& fp)
{
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
}

void OpenniImporter::readData(char* stream)
{
    float intrinsics[9];
    readStream(intrinsics, sizeof(float) * 9, stream);
    intrinsics_ = Eigen::Map<Eigen::Matrix<float, 3, 3> >(intrinsics).cast<double>();

    unsigned short dim[2];
    readStream(dim, sizeof(unsigned short) * 2, stream);

    unsigned short* data = new unsigned short[dim[0] * dim[1]];
    readStream(data, sizeof(unsigned short) * dim[0] * dim[1], stream);

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
}

void OpenniImporter::readCrop(FILE*& fp)
{
    fscanf(fp, "%d%lf%lf%lf%lf", &background_intensity_, &crop_[0], &crop_[1], &crop_[2], &crop_[3]);
    fclose(fp);
}

void OpenniImporter::readCrop(std::istringstream& s)
{
    s >> background_intensity_ >> crop_[0] >> crop_[1] >> crop_[2] >> crop_[3];
}

void OpenniImporter::convertToPointcloud()
{
    pointcloud_.clear();

    const int V = raw_data_.rows();
    const int U = raw_data_.cols();

    // ignore background
    int cols = 0;
    for (int i=0; i<U*V; i++)
    {
        if (raw_data_(i%V, i/V) < background_intensity_)
            cols++;
    }
    if (cols == 0)
        return;

    Eigen::Matrix3Xd B(3, cols);
    int col = 0;
    for (int i=0; i<U*V; i++)
    {
        const int x = i/V;
        const int y = i%V;

        if (raw_data_(i%V, i/V) < background_intensity_ &&
                crop_[0]*U <= x && x <= crop_[1]*U &&
                crop_[2]*V <= y && y <= crop_[3]*V)
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

        if (raw_data_(i%V, i/V) < background_intensity_ &&
                crop_[0]*U <= x && x <= crop_[1]*U &&
                crop_[2]*V <= y && y <= crop_[3]*V)
        {
            pointcloud_.push_back(CP.block(0, col, 3, 1) * raw_data_(i%V, i/V) / 1000.0);
            col++;
        }
    }
}

std::vector<Eigen::Vector3d> OpenniImporter::pointcloud()
{
    return pointcloud_;
}
