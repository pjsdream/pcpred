#ifndef BENCHMARK_GENERATOR_H
#define BENCHMARK_GENERATOR_H


#include <pcpred/feature/human_motion_feature.h>

#include <pcpred/gaussian_process/gpr_multivariate.h>

#include <Eigen/Dense>

#include <string>
#include <vector>


namespace pcpred
{

class BenchmarkGenerator
{
private:

    static const int NUM_ACTIONS = 8;

    struct Edge
    {
        int state;
        int action;
        double prob;

        Edge(int state, int action, double prob)
            : state(state), action(action), prob(prob)
        {
        }
    };

public:

    BenchmarkGenerator();

    void loadScenario(const std::string& filename);
    void generate(const std::string& directory);

    void generateValidationMatrix(const std::string& directory);
    inline Eigen::MatrixXd getEpisodeMatrix() { return episode_matrix_; }
    inline Eigen::VectorXi getEpisodeActions() { return episode_actions_; }

private:

    int num_episodes_;
    int action_set_;
    std::vector<int> num_sequences_;

    std::map<int, std::vector<Edge> > graph_;

    Eigen::MatrixXd episode_matrix_;
    Eigen::VectorXi episode_actions_;
};

}


#endif // BENCHMARK_GENERATOR_H
