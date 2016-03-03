#ifndef FUTURE_MOTION_H
#define FUTURE_MOTION_H


#include <vector>

#include <Eigen/Dense>

#include <pcpred/visualization/marker_array_visualizer.h>

#include <string>


namespace pcpred
{

class FutureMotion
{
private:

    struct Data
    {
        Eigen::Vector3d mu;
        Eigen::Matrix3d sigma;
        double offset;
        double weight;

        Data(){}
        Data(const Eigen::Vector3d& mu, const Eigen::Matrix3d& sigma, double offset, double weight)
            : mu(mu), sigma(sigma), offset(offset), weight(weight) {}
    };

public:

    FutureMotion();

    void setVisualizerTopic(const std::string& topic);

    void loadSavedData(const std::string& filename);

    void getGaussianDistributions(double current_time, double future_time, std::vector<Eigen::Vector3d>& mu, std::vector<Eigen::Matrix3d>& sigma, std::vector<double>& offset, std::vector<double>& weight);

    void visualizePrediction(const std::string& ns, double current_time, double future_time, double probability = 0.95);
    void visualizeHuman();

private:

    int fps_;
    int n_;
    int frames_;
    int future_frames_;

    std::vector<std::vector<std::vector<Data> > > data_;

    MarkerArrayVisualizer* visualizer_;
};

}


#endif // FUTURE_MOTION_H
