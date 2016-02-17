#ifndef LEARNING_MOTION_H
#define LEARNING_MOTION_H


#include <pcpred/feature/human_motion_feature.h>

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

    double input_duration;
    int input_num_pieces;
    double output_duration;
    int output_num_pieces;

    int num_samples;
    int kmeans_k;
    int max_cluster_size;
};


class LearningMotion
{
private:

    struct Stream
    {
        std::string joint_name;
        double t;
        Eigen::Vector3d position;
    };

public:

    LearningMotion();

    void setVerbose(bool flag = true);
    void setOptions(const LearningMotionOptions& options);

    void parseData(const char* filename);
    void learn();

private:

    LearningMotionOptions options_;

    std::vector<Stream> stream_;

    bool verbose_;
};

}


#endif // LEARNING_MOTION_H
