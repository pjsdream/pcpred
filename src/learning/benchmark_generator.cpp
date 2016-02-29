#include <pcpred/learning/benchmark_generator.h>

#include <stdio.h>
#include <string.h>

#include <cstdlib>

using namespace pcpred;


BenchmarkGenerator::BenchmarkGenerator()
{
}

void BenchmarkGenerator::loadScenario(const std::string &filename)
{
    action_set_ = 0;

    FILE* fp = fopen(filename.c_str(), "r");

    char buffer[128];
    while (fscanf(fp, "%s", buffer) == 1)
    {
        if (!strcmp(buffer, "episodes:"))
            fscanf(fp, "%d", &num_episodes_);

        else if (!strcmp(buffer, "graph:"))
        {
            while (fscanf(fp, "%s", buffer) == 1)
            {
                if (buffer[0] == ']')
                    break;

                if (buffer[0] == '[')
                    continue;

                int state = 0;
                int t = 0;
                for (int i=strlen(buffer)-1; i>=0; i--, t++)
                    state |= (buffer[i]-'0') * (1<<t);

                int action;
                double prob;

                fscanf(fp, "%d%lf", &action, &prob);

                graph_[state].push_back( Edge(action == -1 ? -1 : (state | (1 << action)), action, prob) );

                action_set_ |= action;
            }
        }
    }

    fclose(fp);
}

void BenchmarkGenerator::generate(const std::string& directory)
{
    num_sequences_.resize(NUM_ACTIONS);
    for (int i=0; i<NUM_ACTIONS; i++)
    {
        if (action_set_ & (1<<i))
        {
            int n = 0;
            while (true)
            {
                char filename[128];
                sprintf(filename, "%s/../J%02d/sequence%03d.txt", directory.c_str(), i, n);
                FILE* fp = fopen(filename, "r");
                if (fp == NULL)
                {
                    num_sequences_[i] = n-1;
                    break;
                }
                fclose(fp);

                n++;
            }
        }
    }

    for (int e=0; e<num_episodes_; e++)
    {
        char filename[128];
        sprintf(filename, "%s/sequence%03d.txt", directory.c_str(), e);

        FILE* fp = fopen(filename, "w");
        if (fp == NULL)
            return;

        Eigen::MatrixXd episode;
        Eigen::VectorXi episode_actions;

        int state = 0;
        while (state != -1)
        {
            int action = 0;
            int next_state = 0;

            double p = (double)rand() / RAND_MAX;
            for (int i=0; i<graph_[state].size(); i++)
            {
                const double prob = graph_[state][i].prob;
                if (p <= prob)
                {
                    action = graph_[state][i].action;
                    next_state = graph_[state][i].state;
                    break;
                }
                else p -= prob;
            }

            if (action == -1)
                break;

            state = next_state;

            const int sequence = rand() % num_sequences_[action];
            char sequence_filename[128];
            sprintf(sequence_filename, "%s/../J%02d/sequence%03d.txt", directory.c_str(), action, sequence);

            FILE* ifp = fopen(sequence_filename, "r");
            int r, c;
            fscanf(ifp, "%d%d", &r, &c);
            Eigen::MatrixXd motion(r, c);
            for (int i=0; i<r; i++)
            {
                for (int j=0; j<c; j++)
                    fscanf(ifp, "%lf", &motion(i,j));
            }

            episode.conservativeResize(r, episode.cols() + c);
            episode.block(0, episode.cols() - c, r, c) = motion;
            episode_actions.conservativeResize(episode_actions.rows() + c);
            for (int i=0; i<c; i++)
                episode_actions( episode_actions.rows()-c+i ) = action;
        }

        fprintf(fp, "%d %d\n", episode.rows(), episode.cols());
        for (int i=0; i<episode.rows(); i++)
        {
            for (int j=0; j<episode.cols(); j++)
                fprintf(fp, "%lf ", episode(i,j));
            fprintf(fp, "\n");
        }
        for (int i=0; i<episode_actions.rows(); i++)
            fprintf(fp, "%d ", episode_actions(i));
        fprintf(fp, "\n");

        fclose(fp);
    }
}

void BenchmarkGenerator::generateValidationMatrix(const std::string& directory)
{
    num_sequences_.resize(NUM_ACTIONS);
    for (int i=0; i<NUM_ACTIONS; i++)
    {
        if (action_set_ & (1<<i))
        {
            int n = 0;
            while (true)
            {
                char filename[128];
                sprintf(filename, "%s/../V%02d/sequence%03d.txt", directory.c_str(), i, n);
                FILE* fp = fopen(filename, "r");
                if (fp == NULL)
                {
                    num_sequences_[i] = n-1;
                    break;
                }
                fclose(fp);

                n++;
            }
        }
    }

    Eigen::MatrixXd episode;
    Eigen::VectorXi episode_actions;

    int state = 0;
    while (state != -1)
    {
        int action = 0;
        int next_state = 0;

        double p = (double)rand() / RAND_MAX;
        for (int i=0; i<graph_[state].size(); i++)
        {
            const double prob = graph_[state][i].prob;
            if (p <= prob)
            {
                action = graph_[state][i].action;
                next_state = graph_[state][i].state;
                break;
            }
            else p -= prob;
        }

        if (action == -1)
            break;

        state = next_state;

        const int sequence = rand() % num_sequences_[action];
        char sequence_filename[128];
        sprintf(sequence_filename, "%s/../J%02d/sequence%03d.txt", directory.c_str(), action, sequence);

        FILE* ifp = fopen(sequence_filename, "r");
        int r, c;
        fscanf(ifp, "%d%d", &r, &c);
        Eigen::MatrixXd motion(r, c);
        for (int i=0; i<r; i++)
        {
            for (int j=0; j<c; j++)
                fscanf(ifp, "%lf", &motion(i,j));
        }

        episode.conservativeResize(r, episode.cols() + c);
        episode.block(0, episode.cols() - c, r, c) = motion;
        episode_actions.conservativeResize(episode_actions.rows() + c);
        for (int i=0; i<c; i++)
            episode_actions( episode_actions.rows()-c+i ) = action;
    }

    episode_matrix_ = episode;
    episode_actions_ = episode_actions;
}
