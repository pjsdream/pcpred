#ifndef BVH_PREDICTOR_H
#define BVH_PREDICTOR_H


#include <pcpred/prediction/points_predictor.h>
#include <pcpred/import/bvh_importer.h>
#include <pcpred/visualization/marker_array_visualizer.h>

#include <vector>
#include <map>

#include <Eigen/Dense>


namespace pcpred
{

class BvhPredictor
{
private:

    // chi-square-based table for determining axis length of ellipsoid
    static bool gaussian_distribution_radius_table_initialized_;
    static std::map<double, double> gaussian_distribution_radius_table_;

public:

    explicit BvhPredictor(const char* filename);

    // transform of input sequences
    void rotate(double angle, const Eigen::Vector3d& axis);
    void translate(const Eigen::Vector3d& t);

    // parameters for kalman filter. Must not be changed after the frist prediction query
    void setTimestep(double timestep);
    void setSensorDiagonalCovariance(double v);
    void setAccelerationInferenceWindowSize(int window_size);
    void setCollisionProbability(double probability);

    void moveToNextFrame();  // move to next frame (next input stream)
    inline double time() { return time_; }  // current time

    // prediction of next few frames beginning from current time
    void predict(int frame_count);

    // predictino result at a specific future frame as a list of ellipsoids (x-c)^T A (x-c) = 1
    void getPredictedEllipsoids(int frame_number, std::vector<Eigen::Vector3d>& c, std::vector<Eigen::Matrix3d>& A);

    // visualize functions
    void setVisualizerTopic(const char* topic);
    void visualizeHuman();
    void visualizePredictionUpto(int frame_count);

private:

    BvhImporter bvh_importer_;

    // gaussian distribution center predictor
    PointsPredictor* points_predictor_;

    // sphere obstacle size
    std::vector<double> sphere_sizes_;

    double timestep_;
    double time_;

    double collision_probability_;

    MarkerArrayVisualizer* visualizer_;
};

}


#endif // BVH_PREDICTOR_H
