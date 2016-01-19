#ifndef POINTCLOUD_HUMAN_PREDICTOR_H
#define POINTCLOUD_HUMAN_PREDICTOR_H


/*
 * point cloud data to human shape recognition
 * based on the paper: Real-Time Human Pose Tracking From Range Data
 * simplified model
 */


#include <pcpred/pointcloud/pointcloud.h>
#include <pcpred/shape/human.h>
#include <pcpred/prediction/points_predictor.h>
#include <pcpred/visualization/marker_array_visualizer.h>

#include <Eigen/Dense>

#include <vector>


namespace pcpred
{

class PointcloudHumanPredictor
{
public:

    PointcloudHumanPredictor();

    // optimization parameters
    inline void setMaximumIterations(int iteration) { max_iteration_ = iteration; }
    inline void setGradientDescentMaximumIterations(int iteration) { gradient_descent_max_iteration_ = iteration; }
    inline void setGradientDescentAlpha(double alpha) { gradient_descent_alpha_ = alpha; }
    void setHumanShapeLengthConstraintEpsilon(double epslion);

    // prediction parameters
    void setTimestep(double timestep);
    void setSensorDiagonalCovariance(double v);
    void setCollisionProbability(double p);
    void setAccelerationInferenceWindowSize(double window_size);

    int numSpheres();

    void loadHumanShapeFromFile(const char* filename);

    void observe(const Eigen::Vector3d& camera_position, const Pointcloud& pointcloud); // redirects to function using a list of centers
    void observe(const Eigen::Vector3d& camera_position, const std::vector<Eigen::Vector3d>& pointcloud);

    void predict(int frame_count);

    void getPredictionResult(int frame_number, int sphere_index, Eigen::Vector3d& mu, Eigen::Matrix3d& sigma);
    void getPredictionResults(int frame_number, std::vector<Eigen::Vector3d>& mu, std::vector<Eigen::Matrix3d>& sigma);
    void getPredictedEllipsoids(int frame_number, std::vector<Eigen::Vector3d>& c, std::vector<Eigen::Matrix3d>& A);

    // visualize functions
    void setVisualizerTopic(const char* topic);
    void visualizeHuman();
    void visualizePredictionUpto(const int frame_count);

private:

    void optimizeHumanShape(const Eigen::Vector3d& camera_position, const std::vector<Eigen::Vector3d>& pointcloud);

    Human human_shape_;
    std::vector<double> sphere_radius_;

    PointsPredictor* predictor_;

    int max_iteration_;
    int gradient_descent_max_iteration_;
    double gradient_descent_alpha_;

    int capsule_divisor_;

    double collision_probability_;

    MarkerArrayVisualizer* visualizer_;
};

}


#endif // POINTCLOUD_HUMAN_PREDICTOR_H
