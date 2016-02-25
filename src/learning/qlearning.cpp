#include <pcpred/learning/qlearning.h>

#include <std_msgs/Int32.h>

#include <stdio.h>

using namespace pcpred;


QLearningOptions::QLearningOptions()
{
    alpha = 0.1;
    gamma = 0.5;
    epsilon = 0.1;
}

void QLearningOptions::parse(const std::string& filename)
{
    FILE* fp = fopen(filename.c_str(), "r");

    char variable[128];
    while (fscanf(fp, "%s", variable) == 1)
    {
        if (!strcmp(variable, "alpha:"))
        {
            fscanf(fp, "%lf", &alpha);
        }
        else if (!strcmp(variable, "gamma:"))
        {
            fscanf(fp, "%lf", &gamma);
        }
        else if (!strcmp(variable, "epsilon:"))
        {
            fscanf(fp, "%lf", &epsilon);
        }
    }

    fclose(fp);
}

void QLearningOptions::print()
{
    printf("Q-learning alpha   = %lf\n", alpha);
    printf("Q-learning gamma   = %lf\n", gamma);
    printf("Q-learning epsilon = %lf\n", epsilon);
}


QLearning::QLearning()
{
    ros::NodeHandle nh;
    planning_request_publisher_ = nh.advertise<std_msgs::Int32>("planning_request", 100);
    planning_time_subscriber_ = nh.subscribe<std_msgs::Float64>("planning_time", 100, &QLearning::callbackPlanningTime, this);
    ros::Duration(1.0).sleep();
}

void QLearning::setOptions(const QLearningOptions &options)
{
    options_ = options;
}

void QLearning::loadData(const std::string &filename)
{
    FILE* fp = fopen(filename.c_str(), "r");

    fscanf(fp, "%d%d", &num_states_, &num_actions_);

    transition_.resize(num_states_, num_actions_);
    for (int i=0; i<num_states_; i++)
    {
        for (int j=0; j<num_actions_; j++)
        {
            transition_(i,j) = -1;
            fscanf(fp, "%d", &transition_(i,j));
        }
    }

    q_table_.resize(num_states_, num_actions_);
    for (int i=0; i<num_states_; i++)
    {
        for (int j=0; j<num_actions_; j++)
        {
            q_table_(i,j) = 0.0;
            fscanf(fp, "%lf", &q_table_(i,j));
        }
    }

    h_table_.resize(num_states_, num_actions_);
    for (int i=0; i<num_states_; i++)
    {
        for (int j=0; j<num_actions_; j++)
        {
            h_table_(i,j) = 0.0;
            fscanf(fp, "%lf", &h_table_(i,j));
        }
    }

    fclose(fp);
}

void QLearning::saveData(const std::string &filename)
{
    FILE* fp = fopen(filename.c_str(), "w");

    fprintf(fp, "%d %d\n\n", num_states_, num_actions_);

    for (int i=0; i<num_states_; i++)
    {
        for (int j=0; j<num_actions_; j++)
            fprintf(fp, "%d ", transition_(i,j));
        fprintf(fp, "\n");
    }
    fprintf(fp, "\n");

    for (int i=0; i<num_states_; i++)
    {
        for (int j=0; j<num_actions_; j++)
            fprintf(fp, "%lf ", q_table_(i,j));
        fprintf(fp, "\n");
    }
    fprintf(fp, "\n");

    for (int i=0; i<num_states_; i++)
    {
        for (int j=0; j<num_actions_; j++)
            fprintf(fp, "%lf ", h_table_(i,j));
        fprintf(fp, "\n");
    }
    fprintf(fp, "\n");

    fclose(fp);
}

double QLearning::computeReward(int state, int action)
{
    double r = 0.0;

    // user defined reward
    r += h_table_(state, action);

    // delay penalty
    std_msgs::Int32 planning_action;
    planning_action.data = action;
    ROS_INFO("Planning request %d", planning_action.data);
    planning_request_publisher_.publish(planning_action);

    ROS_INFO("Waiting for planning time");
    while (ros::ok() && planning_time_queue_.empty()) ros::spinOnce();
    const double planning_time = planning_time_queue_.front();
    planning_time_queue_.pop();
    ROS_INFO("Planning time %lf", planning_time);
    r -= planning_time;

    return r;
}

void QLearning::reinforcementLearn()
{
    reinforcementLearn( rand() % num_states_ );
}

void QLearning::reinforcementLearn(int state)
{
    int action;

    const double x = (double)rand() / RAND_MAX;
    if (x <= options_.epsilon)
    {
        action = rand() % num_actions_;
    }
    else
    {
        action = 0;
        for (int i=1; i<num_actions_; i++)
        {
            if (transition_(state, i) != -1 && (transition_(state, action) == -1 || q_table_(state, i) > q_table_(state, action)))
                action = i;
        }
    }

    reinforcementLearn(state, action);
}

void QLearning::reinforcementLearn(int state, int action)
{
    if (transition_(state, action) == -1)
        return;

    const double r = computeReward(state, action);

    q_table_(state, action) += options_.alpha * (r + options_.gamma * q_table_.row( transition_(state, action) ).maxCoeff() - q_table_(state, action));
}

void QLearning::callbackPlanningTime(const std_msgs::Float64::ConstPtr& msg)
{
    planning_time_queue_.push(msg->data);
}
