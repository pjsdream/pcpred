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
    double alpha;   // learning rate
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

    void loadData(const std::string& filename);
    void saveData(const std::string& filename);

    void reinforcementLearn();
    void reinforcementLearn(int state);
    void reinforcementLearn(int state, int action);

    double computeReward(int state, int action);

private:

    void callbackPlanningTime(const std_msgs::Float64::ConstPtr& msg);

    QLearningOptions options_;

    int num_states_;
    int num_actions_;

    Eigen::VectorXi transition_;

    Eigen::VectorXd q_table_;
    Eigen::VectorXd h_table_;

    ros::Publisher planning_request_publisher_;
    ros::Subscriber planning_time_subscriber_;
    std::queue<int> planning_time_queue_;
};

}


#endif // KMEANS_H
