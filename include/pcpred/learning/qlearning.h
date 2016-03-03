#ifndef Q_LEARNING_H
#define Q_LEARNING_H


#include <Eigen/Dense>

#include <vector>
#include <queue>

#include <ros/ros.h>
#include <std_msgs/Float64.h>


namespace pcpred
{

struct QLearningOptions
{
    double omega;   // learning rate
    double gamma;   // discount factor
    double epsilon; // action selection strategy (default probability among all actions)

    QLearningOptions();

    void parse(const std::string& filename);
    void print();
};

class QLearning
{
public:

    QLearning();

    void setOptions(const QLearningOptions& options);

    inline void setState(int state) { state_ = state; }
    inline int getLastAction() { return last_action_; }
    inline double getCompletionTime() { return completion_time_; }

    void loadData(const std::string& filename);
    void saveData(const std::string& filename);

    int reinforcementLearn();
    int reinforcementLearn(int state);
    int reinforcementLearn(int state, int action);

    double computeReward(int state, int action);

private:

    void callbackPlanningTime(const std_msgs::Float64::ConstPtr& msg);

    QLearningOptions options_;

    int num_states_;
    int num_actions_;

    double completion_time_;

    int state_;
    int last_action_;

    Eigen::MatrixXi transition_;

    Eigen::MatrixXd q_table_;
    Eigen::MatrixXd h_table_;

    ros::Publisher planning_request_publisher_;
    ros::Subscriber planning_time_subscriber_;
    std::queue<double> planning_time_queue_;

    int timestep_;
};

}


#endif // KMEANS_H
