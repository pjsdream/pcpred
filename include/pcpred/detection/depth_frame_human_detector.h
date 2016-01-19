#ifndef DEPTH_FRAME_HUMAN_DETECTOR_H
#define DEPTH_FRAME_HUMAN_DETECTOR_H


#include <pcpred/shape/human.h>
#include <pcpred/visualization/marker_array_visualizer.h>

#include <Eigen/Dense>


namespace pcpred
{

class DepthFrameHumanDetector
{
private:

    static const int background_depth_ = 65535;

    static Eigen::MatrixXd medianFiltering(const Eigen::MatrixXd& depth_frame);
    static std::vector<Eigen::Vector3d> get3DPointCloudFromDepthFrame(const Eigen::MatrixXd& depth_frame, const Eigen::Matrix4d& intrinsic, const Eigen::Matrix4d& extrinsic);

public:

    DepthFrameHumanDetector();

    inline void setMaxIterations(int iterations) { max_iterations_ = iterations; }
    inline void setMaxGradientDescentIterations(int iterations) { max_gradient_descent_iterations_ = iterations; }
    inline void setMaxProjectionIterations(int iterations) { max_projection_iterations_ = iterations; }
    inline void setGradientDescentAlpha(double alpha) { gradient_descent_alpha_ = alpha; }
    inline void setProjectionAlpha(double alpha) { projection_alpha_ = alpha; }
    inline void setLengthConstraintEpsilon(double epsilon) { length_constraint_epsilon_ = epsilon; }
    inline void setCapsuleDivisor(int divisor) { capsule_divisor_ = divisor; }

    void loadHumanShapeFromFile(const char* filename);

    void observeDepthFrame(Eigen::MatrixXd depth_frame, const Eigen::Matrix4d& intrinsic, const Eigen::Matrix4d& extrinsic);

    void setVisualizerTopic(const char* topic);
    void visualizeHuman();

private:

    Human human_shape_;

    Eigen::MatrixXd last_depth_frame_;

    int max_iterations_;
    int max_gradient_descent_iterations_;
    int max_projection_iterations_;
    double gradient_descent_alpha_;
    double projection_alpha_;
    double length_constraint_epsilon_;

    int capsule_divisor_;

    MarkerArrayVisualizer* visualizer_;
};

}


#endif // DEPTH_FRAME_HUMAN_DETECTOR_H
