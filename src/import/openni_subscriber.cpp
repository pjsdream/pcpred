#include <pcpred/import/openni_subscriber.h>

#include <stdio.h>
#include <iostream>

using namespace pcpred;


static void callbackRawImage(const sensor_msgs::Image::ConstPtr& msg)
{
    OpenniSubscriber* subscriber = OpenniSubscriber::getInstance();
    subscriber->onReceivedRawImage(msg);
}

static void callbackCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    OpenniSubscriber* subscriber = OpenniSubscriber::getInstance();
    subscriber->onReceivedCameraInfo(msg);
}


OpenniSubscriber::OpenniSubscriber()
{
    print_message_flag_ = false;
    message_received_ = false;
    is_recording_ = false;

    raw_data_.resize(640, 480);

    // ros subscriber setup
    ros::NodeHandle n;
    raw_image_subscriber_ = n.subscribe("camera/depth/image_raw", 1000, callbackRawImage);
    camera_info_subscriber_ = n.subscribe("camera/depth/camera_info", 1000, callbackCameraInfo);
}


void OpenniSubscriber::onReceivedRawImage(const sensor_msgs::Image::ConstPtr& msg)
{
    message_received_ = true;

    if (print_message_flag_)
    {
        std::cout << "Received [raw image] message" << std::endl;
        std::cout << "width  = " << msg->width << std::endl;
        std::cout << "height = " << msg->height << std::endl;
        std::cout << "step   = " << msg->step << std::endl;
        std::cout << "is big endian? " << (int)msg->is_bigendian << std::endl;
        std::cout << "encoding = " << msg->encoding << std::endl;
        std::cout.flush();
    }

    const int height = msg->height;
    const int width = msg->width;
    const int size = msg->step / width;

    if (!msg->is_bigendian && msg->encoding == "16UC1")
    {
        raw_data_.resize(height, width);
        for (int i=0; i<height; i++)
        {
            for (int j=0; j<width; j++)
                raw_data_(i, j) = *(unsigned short*)(void*)&(msg->data[(i * width + j) * size]);
        }
    }

    else
    {
        ROS_WARN("Not supported depth image encoding");
    }

    if (is_recording_)
        recorded_raw_data_.push_back(raw_data_);
}

void OpenniSubscriber::onReceivedCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    message_received_ = true;

    if (print_message_flag_)
    {
        std::cout << "Received [camera info] message" << std::endl;
        std::cout << "width  = " << msg->width << std::endl;
        std::cout << "height = " << msg->height << std::endl;
        std::cout << "distortion model = " << msg->distortion_model << std::endl;

        std::cout << "D = [ ";
        for (int i=0; i<5; i++)
            std::cout << msg->D[i] << ' ';
        std::cout << "]" << std::endl;

        std::cout << "K = [ ";
        for (int i=0; i<9; i++)
            std::cout << msg->K[i] << ' ';
        std::cout << "]" << std::endl;

        std::cout << "R = [ ";
        for (int i=0; i<9; i++)
            std::cout << msg->R[i] << ' ';
        std::cout << "]" << std::endl;

        std::cout << "P = [ ";
        for (int i=0; i<12; i++)
            std::cout << msg->P[i] << ' ';
        std::cout << "]" << std::endl;

        std::cout << "binning = [" << msg->binning_x << " " << msg->binning_y << "]" << std::endl;
        std::cout << "ROI = [" << msg->roi.width << " " << msg->roi.height << "]" << std::endl;

        std::cout.flush();
    }

    // assuming no distortion, no binning, no ROI
    // assuming monocular camera (i.e. R = I, P[1:3][1:3] = K, P[1:3][4] = 0)
    // matrices are row-major ordered

    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            intrinsics_(i, j) = msg->K[i*3 + j];

    if (is_recording_)
        recorded_intrinsics_.push_back(intrinsics_);
}


void OpenniSubscriber::readDepthFrame()
{
    const double background_intensity = 2800;

    // In this current implementation, the timestamps from raw image and camera info may not be matched
    while (!message_received_)
        ros::spinOnce();
    message_received_ = false;

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
        return;

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
}

std::vector<Eigen::Vector3d> OpenniSubscriber::pointcloud()
{
    return pointcloud_;
}


void OpenniSubscriber::record(int frame_count, int sequence_number)
{
    is_recording_ = true;

    int frame = 0;
    while (frame < frame_count)
    {
        recorded_raw_data_.clear();
        recorded_intrinsics_.clear();
        ros::spinOnce();

        const int recorded_frames = std::min(recorded_raw_data_.size(), recorded_intrinsics_.size());

        int count = 0;
        while (count < recorded_frames && frame < frame_count)
        {
            printf("recording %d/%d\n", frame, frame_count);
            char filename[128];
            sprintf(filename, "../data/C%d/image%04d.txt", sequence_number, frame);
            FILE* fp = fopen(filename, "wb");

            // intrinsics
            float intrinsics[9];
            for (int i=0; i<3; i++)
                for (int j=0; j<3; j++)
                    intrinsics[i + j*3] = recorded_intrinsics_[count](i, j);
            fwrite(intrinsics, sizeof(float), 9, fp);

            const unsigned short dim[2] = {recorded_raw_data_[count].rows(), recorded_raw_data_[count].cols()};
            fwrite(dim, sizeof(unsigned short), 2, fp);

            std::vector<unsigned short> v;
            for (int i=0; i<dim[0]; i++)
            {
                for (int j=0; j<dim[1]; j++)
                    v.push_back( recorded_raw_data_[count](i, j) );
            }
            fwrite(&v[0], sizeof(unsigned short), v.size(), fp);

            fclose(fp);

            frame++;
            count++;
        }
    }
}
