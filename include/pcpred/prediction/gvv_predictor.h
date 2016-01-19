#ifndef GVV_PREDICTOR_H
#define GVV_PREDICTOR_H


#include <pcpred/import/gvv_data_importer.h>
#include <pcpred/prediction/pointcloud_human_predictor.h>
#include <pcpred/visualization/pointcloud_visualizer.h>
#include <pcpred/pointcloud/pointcloud.h>

#include <Eigen/Dense>


namespace pcpred
{

class GvvPredictor
{
private:

    static const double motion_timestep_;  // 0.033

public:

    GvvPredictor(int sequence_number);

    // transform of input sequences
    void rotate(double angle, const Eigen::Vector3d& axis);
    void translate(const Eigen::Vector3d& t);

    // parameters for kalman filter. Must not be changed after the frist prediction query
    void setTimestep(double timestep);
    void setSensorDiagonalCovariance(double v);
    void setAccelerationInferenceWindowSize(int window_size);
    void setCollisionProbability(double probability);

    // parameters for human detection
    void setMaximumIterations(int iterations);
    void setGradientDescentMaximumIterations(int iterations);
    void setGradientDescentAlpha(double alpha);
    void setHumanShapeLengthConstraintEpsilon(double epsilon);

    void moveToNextFrame();
    inline double time() { return time_; }

    // prediction of next few frames beginning from current time
    void predict(int frame_count);

    // prediction result at a specific future frame as a list of ellipsoids
    void getPredictedGaussianDistribution(int frame_number, std::vector<Eigen::Vector3d>& mu, std::vector<Eigen::Matrix3d>& sigma, std::vector<double>& radius);

    void setVisualizerTopic(const char* topic);
    void visualizePointcloud();
    void visualizeHuman();
    void visualizePredictionUpto(int frame_count);

private:

    GVVDataImporter importer_;
    Eigen::Affine3d transformation_;

    PointcloudHumanPredictor predictor_;

    Pointcloud pointcloud_;

    int sequence_number_;
    int last_imported_frame_;

    double timestep_;
    double time_;

    double collision_probability_;

    PointcloudVisualizer* pointcloud_visualizer_;
};

}


#endif // GVV_PREDICTOR_H
