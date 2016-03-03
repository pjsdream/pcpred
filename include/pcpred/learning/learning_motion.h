#ifndef LEARNING_MOTION_H
#define LEARNING_MOTION_H


#include <pcpred/util/logger.h>
#include <pcpred/feature/human_motion_feature.h>
#include <pcpred/gaussian_process/gpr_multivariate.h>

#include <pcpred/visualization/marker_array_visualizer.h>

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
    void infer(const Eigen::VectorXd& feature, int state, int last_action);

    void loadTrainedData(const std::string& filename);
    void saveTrainedData(const std::string& filename);
    void saveFutureMotion(const std::string& filename);

    void setVisualizerTopic(const std::string& topic);
    void visualizeInferenceResult();

private:

    void parseData(const char* directory);

    void appendColumn(Eigen::MatrixXd& m, const Eigen::VectorXd& x);

    LearningMotionOptions options_;
    std::string human_model_filename_;
    HumanMotionFeature feature_template_;

    // input episodes
    std::vector<Eigen::MatrixXd> ksi_;
    std::vector<Eigen::VectorXi> actions_;
    std::vector<Eigen::VectorXi> progress_states_;
    int num_action_types_;

    std::vector<std::vector<Eigen::VectorXd> > cluster_centers_;   // [progress_state][cluster_id](vector_index)
    std::vector<std::vector<GprMultivariate> > gprs_;
    std::vector<std::vector<Eigen::MatrixXd> > gpr_outputs_;  // [cluster](row:channel, column:data)

    std::vector<double> current_action_prob_; // [state]
    std::vector<Eigen::VectorXd> means_;      // [action](channel)
    std::vector<Eigen::VectorXd> variances_;

    MarkerArrayVisualizer* visualizer_;
};

}


#endif // LEARNING_MOTION_H
