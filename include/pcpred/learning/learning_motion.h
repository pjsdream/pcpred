#ifndef LEARNING_MOTION_H
#define LEARNING_MOTION_H


#include <pcpred/util/logger.h>

#include <pcpred/feature/human_motion_feature.h>

#include <pcpred/gaussian_process/gpr_multivariate.h>

#include <Eigen/Dense>

#include <string>
#include <vector>


namespace pcpred
{

class LearningMotionOptions
{
public:

    LearningMotionOptions();

    void parse(const char* filename);
    void print();

public:

    int input_frames;
    int output_frames;

    int num_samples;
    int kmeans_k;
    int max_cluster_size;

    double gpr_l;
    double gpr_sigma_f;
    double gpr_sigma_n;
};


class LearningMotion : public Logger
{
public:

    LearningMotion();

    void setOptions(const LearningMotionOptions& options);
    void setHumanModelFilename(const std::string& filename);

    void learn(const char *directory);

    void loadTrainedData(const std::string& filename);
    void saveTrainedData(const std::string& filename);

private:

    void parseData(const char* directory);

    LearningMotionOptions options_;
    std::string human_model_filename_;

    std::vector<Eigen::MatrixXd> ksi_;
    std::vector<Eigen::VectorXi> actions_;
    std::vector<Eigen::VectorXi> progress_states_;

    std::vector<Eigen::VectorXd> cluster_centers_;
    std::vector<GprMultivariate> gprs_;
    std::vector<Eigen::MatrixXd> gpr_outputs_;  // [cluster](row:data, column:channel)
};

}


#endif // LEARNING_MOTION_H
