#ifndef MOTION_PREDICTOR_H
#define MOTION_PREDICTOR_H


#include <pcpred/learning/learning_motion.h>
#include <pcpred/feature/human_motion_feature.h>
#include <pcpred/gaussian_process/gpr_multivariate.h>
#include <pcpred/util/logger.h>
#include <pcpred/visualization/marker_array_visualizer.h>


namespace pcpred
{

class MotionPredictor : public Logger
{
private:

    static void appendColumn(Eigen::MatrixXd& matrix, const Eigen::VectorXd& column);
    static void blendMotion(Eigen::MatrixXd& motion, int frame, int motion_blend_frames);

public:

    MotionPredictor();

    void setVisualizerTopic(const std::string& topic);

    void initialize(const std::string& data_directory, int benchmark_number);
    void setActionAvailability(int action, bool availability);

    void moveToFuture(double time);

    void getPredictedEllipsoids(double time, std::vector<Eigen::Vector3d>& mu, std::vector<Eigen::Matrix3d>& sigma, std::vector<double>& offsets, std::vector<double>& weights);
    int getNextRobotAction();

    void visualizeFutureMotion();
    void visualizeHuman();

private:

    void initializeActionOrder();
    void initializeIdleMotion();
    void parseData();
    void learn();

    void predictFutureMotion();

    void appendIdle();
    void appendAction(int action);
    void appendActionSequence(int action, int sequence);
    void appendActionLastFrames(int action, int sequence, int last_frames = -1, bool is_idle = false);

    static const int motion_blend_frames_ = 7;
    static const int num_action_types_ = 4;
    static const int fps_ = 15;
    std::string data_directory_;
    int benchmark_number_;

    LearningMotionOptions options_;

    // benchmark sequences
    int num_episodes_;
    std::vector<Eigen::MatrixXd> ksi_;
    std::vector<Eigen::VectorXi> actions_;
    std::vector<Eigen::VectorXi> progress_states_;

    // trained data
    HumanMotionFeature feature_template_;
    std::vector<std::vector<GprMultivariate> > gprs_;
    std::vector<std::vector<Eigen::MatrixXd> > gpr_outputs_;
    std::vector<std::vector<Eigen::VectorXd> > cluster_centers_;
    std::vector<std::vector<double> > next_action_histogram_;

    // inference results
    std::vector<double> desired_action_prob_;
    std::vector<double> current_action_prob_;
    std::vector<Eigen::VectorXd> means_;
    std::vector<Eigen::VectorXd> variances_;

    // number of files for each action sequence
    std::vector<int> num_sequences_;

    // current states
    double remaining_time_;
    int current_ksi_column_;
    Eigen::MatrixXd current_ksi_;
    Eigen::VectorXi current_actions_;
    HumanMotionFeature current_feature_;
    int state_; // current human progress state
    int last_action_; // last human action
    int action_order_index_;
    std::vector<int> action_order_; // human action order
    int action_availability_; // available human actions

    MarkerArrayVisualizer* visualizer_;
};

}


#endif // MOTION_PREDICTOR_H
