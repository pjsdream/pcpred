#include <pcpred/learning/learning_motion.h>

#include <pcpred/learning/hierarchical_kmeans.h>

#include <stdio.h>
#include <string.h>

#include <cstdlib>

using namespace pcpred;


LearningMotionOptions::LearningMotionOptions()
{
    input_frames = 15;
    output_frames = 15;
    num_samples = 100000;
    max_cluster_size = 100;
    kmeans_k = 3;
}

void LearningMotionOptions::parse(const char* filename)
{
    FILE* fp = fopen(filename, "r");
    if (fp == NULL)
        return;

    char param[128];
    while (fscanf(fp, "%s", param) == 1)
    {
        if (!strcmp(param, "input_frames:"))
            fscanf(fp, "%lf", &input_frames);

        else if (!strcmp(param, "output_frames:"))
            fscanf(fp, "%lf", &output_frames);

        else if (!strcmp(param, "num_samples:"))
            fscanf(fp, "%d", &num_samples);

        else if (!strcmp(param, "max_cluster_size:"))
            fscanf(fp, "%d", &max_cluster_size);

        else if (!strcmp(param, "kmeans_k:"))
            fscanf(fp, "%d", &kmeans_k);
    }

    fclose(fp);
}

void LearningMotionOptions::print()
{
    printf("# frames         = %d(input)  %d(output)\n", input_frames, output_frames);
    printf("# samples        = %d\n", num_samples);
    printf("max cluster size = %d\n", max_cluster_size);
    printf("k (k-means)      = %d\n", kmeans_k);
}


LearningMotion::LearningMotion()
{
}

void LearningMotion::setVerbose(bool flag)
{
    verbose_ = flag;
}

void LearningMotion::setOptions(const LearningMotionOptions& options)
{
    options_ = options;
}

void LearningMotion::setHumanModelFilename(const std::string& filename)
{
    human_model_filename_ = filename;
}

void LearningMotion::parseData(const char *directory)
{
    int e = 0;
    while (true)
    {
        char filename[128];
        sprintf(filename, "%s/sequence%03d.txt", directory, e);

        FILE* fp = fopen(filename, "r");
        if (fp == NULL)
            return;

        int r, c;
        fscanf(fp, "%d%d", &r, &c);

        Eigen::MatrixXd ksi;
        for (int i=0; i<r; i++)
        {
            for (int j=0; j<c; j++)
                fscanf(fp, "%lf", &ksi(i,j));
        }
        ksi_.push_back(ksi);

        Eigen::VectorXi actions;
        Eigen::VectorXi progress_states;
        actions.resize(c);
        progress_states.resize(c);
        for (int i=0; i<c; i++)
        {
            fscanf(fp, "%lf", &actions(i));

            progress_states(i) = 1 << actions(i);
            if (i)
                progress_states(i) |= progress_states(i-1);
        }
        actions_.push_back(actions);
        progress_states_.push_back(progress_states);

        fclose(fp);
    }
}

void LearningMotion::learn(const char *directory)
{
    HumanMotionFeature input_feature;
    input_feature.loadHumanJoints(human_model_filename_);

    HumanMotionFeature output_feature;
    output_feature.loadHumanJoints(human_model_filename_);

    Eigen::MatrixXd X(input_feature.columnFeatureSize(), options_.num_samples);
    Eigen::MatrixXd Y(output_feature.columnFeatureSize(), options_.num_samples);

    if (verbose_)
    {
        printf("input feature dimension = %d\n", input_feature.columnFeatureSize());
        fflush(stdout);
    }


    int num_possibilities;
    for (int i=0; i<ksi_.size(); i++)
        num_possibilities += ksi_[i].cols();

    // data sampling from episodes
    for (int i=0; i<options_.num_samples; i++)
    {
        int r = rand() % num_possibilities;
        int demo;
        int s;
        for (int j=0; j<ksi_.size(); j++)
        {
            if (r < ksi_[i].cols() - options_.input_frames - options_.output_frames)
            {
                demo = j;
                s = options_.input_frames + r - 1;
                break;
            }
            r -= ksi_[i].cols() - options_.input_frames - options_.output_frames;
        }

        input_feature.clearFeature();
        output_feature.clearFeature();

        for (int j = s - options_.input_frames + 1; j <= s; j++)
            input_feature.addFrame(ksi_[demo].col(j));
        for (int j = s+1; j <= s + options_.output_frames; j++)
            output_feature.addFrame(ksi_[demo].col(j));

        X.col(i) = input_feature.columnFeature();
        Y.col(i) = output_feature.columnFeature();
    }

    if (verbose_)
    {
        HierarchicalKmeans kmeans;
        kmeans.setVerbose(verbose_);
        kmeans.setSizeLimit( options_.max_cluster_size );
        kmeans.setK( options_.kmeans_k );
        kmeans.setTerminationCondition(100000);

        printf("hierarchical K-means clustering started\n"); fflush(stdout);
        const std::vector<int> cluster = kmeans.clusterSizeConstraint(X);
        printf("hierarchical K-means clustering finished\n"); fflush(stdout);


        std::map<int, int> cluster_size;
        for (int i=0; i<cluster.size(); i++)
            cluster_size[ cluster[i] ]++;

        for (int i=0; i<cluster.size(); i++)
            cluster_centers_[ cluster[i] ] += X.col(i) / cluster_size[ cluster[i] ];

        printf("GPR started\n"); fflush(stdout);
        for (int i=0; i<cluster_size.size(); i++)
        {
            gprs_[i].setHyperParameters(gpr_l_, gpr_sigma_f_, gpr_sigma_n_);

            Eigen::MatrixXd cX(X.cols(), cluster_size[i]);

            int c = 0;
            for (int j=0; j<cluster.size(); j++)
            {
                if (cluster[j] == i)
                    cX.col(c++) = X.col(j);
            }

            gprs_[i].setObservationInput(cX);
        }
        printf("GPR finished\n"); fflush(stdout);
    }
}
