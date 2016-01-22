#ifndef OPENNI_SUBSCRIBER_H
#define OPENNI_SUBSCRIBER_H


#include <pcpred/util/singleton.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <ros/ros.h>

#include <vector>

#include <Eigen/Dense>


namespace pcpred
{

class OpenniSubscriber : public Singleton<OpenniSubscriber>
{
public:

    OpenniSubscriber();

    void onReceivedRawImage(const sensor_msgs::Image::ConstPtr& msg);
    void onReceivedCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg);

    // for debug
    inline void setPrintMessages(bool flag = true) { print_message_flag_ = flag; }

    void readDepthFrame();
    std::vector<Eigen::Vector3d> pointcloud();

    void record(int frame_count, int sequence_number);

private:

    ros::Subscriber raw_image_subscriber_;
    ros::Subscriber camera_info_subscriber_;

    Eigen::MatrixXd raw_data_;

    Eigen::Matrix3d intrinsics_;

    std::vector<Eigen::Vector3d> pointcloud_;

    bool print_message_flag_;
    bool message_received_;

    bool is_recording_;
    std::vector<Eigen::MatrixXd> recorded_raw_data_;
    std::vector<Eigen::Matrix3d> recorded_intrinsics_;
};

}


#endif // OPENNI_SUBSCRIBER_H
