#include <pcpred/learning/learning_motion.h>

#include <pcpred/learning/hierarchical_kmeans.h>

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

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
            fscanf(fp, "%d", &input_frames);

        else if (!strcmp(param, "output_frames:"))
            fscanf(fp, "%d", &output_frames);

        else if (!strcmp(param, "num_samples:"))
            fscanf(fp, "%d", &num_samples);

        else if (!strcmp(param, "max_cluster_size:"))
            fscanf(fp, "%d", &max_cluster_size);

        else if (!strcmp(param, "kmeans_k:"))
            fscanf(fp, "%d", &kmeans_k);

        else if (!strcmp(param, "gpr_l:"))
            fscanf(fp, "%lf", &gpr_l);

        else if (!strcmp(param, "gpr_sigma_f:"))
            fscanf(fp, "%lf", &gpr_sigma_f);

        else if (!strcmp(param, "gpr_sigma_n:"))
            fscanf(fp, "%lf", &gpr_sigma_n);
    }

    fclose(fp);
}

void LearningMotionOptions::print()
{
    printf("# frames         = %d(input)  %d(output)\n", input_frames, output_frames);
    printf("# samples        = %d\n", num_samples);
    printf("max cluster size = %d\n", max_cluster_size);
    printf("k (k-means)      = %d\n", kmeans_k);
    printf("gpr l, sigmas    = %lf %lf %lf\n", gpr_l, gpr_sigma_f, gpr_sigma_n);
}


LearningMotion::LearningMotion()
{
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

        Eigen::MatrixXd ksi(r, c);
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
            fscanf(fp, "%d", &actions(i));

            progress_states(i) = 1 << actions(i);
            if (i)
                progress_states(i) |= progress_states(i-1);
        }
        actions_.push_back(actions);
        progress_states_.push_back(progress_states);

        fclose(fp);

        e++;
    }
}

void LearningMotion::learn(const char *directory)
{
    parseData(directory);

    HumanMotionFeature input_feature;
    input_feature.loadHumanJoints(human_model_filename_);

    HumanMotionFeature output_feature;
    output_feature.loadHumanJoints(human_model_filename_);

    Eigen::MatrixXd X(ksi_[0].rows() * options_.input_frames, options_.num_samples);
    Eigen::MatrixXd Y(ksi_[0].rows() * options_.output_frames, options_.num_samples);


    LOG("dimension: %d %d\n", X.rows(), X.cols());


    int num_possibilities = 0;
    for (int i=0; i<ksi_.size(); i++)
        num_possibilities += ksi_[i].cols() - options_.input_frames - options_.output_frames;

    // data sampling from episodes
    for (int i=0; i<options_.num_samples; i++)
    {
        int r = rand() % num_possibilities;
        int demo;
        int s;
        for (int j=0; j<ksi_.size(); j++)
        {
            if (r < ksi_[j].cols() - options_.input_frames - options_.output_frames)
            {
                demo = j;
                s = options_.input_frames + r - 1;
                break;
            }
            r -= ksi_[j].cols() - options_.input_frames - options_.output_frames;
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


    HierarchicalKmeans kmeans;
    kmeans.setVerbose(verbose_);
    kmeans.setSizeLimit( options_.max_cluster_size );
    kmeans.setK( options_.kmeans_k );
    kmeans.setTerminationCondition(100000);

    LOG("hierarchical K-means clustering started\n");
    const std::vector<int> cluster = kmeans.clusterSizeConstraint(X);
    LOG("hierarchical K-means clustering finished\n");


    std::map<int, int> cluster_size;
    for (int i=0; i<cluster.size(); i++)
        cluster_size[ cluster[i] ] = 0;
    for (int i=0; i<cluster.size(); i++)
        cluster_size[ cluster[i] ]++;

    if (verbose_)
    {
        LOG("cluster size %d\n", cluster.size());
        for (std::map<int, int>::iterator it = cluster_size.begin(); it != cluster_size.end(); it++)
            LOG("cluster %d: %d\n", it->first, it->second);
    }

    cluster_centers_.resize( cluster_size.size(), Eigen::VectorXd(X.rows()) );
    for (int i=0; i<cluster.size(); i++)
        cluster_centers_[ cluster[i] ] += X.col(i) / cluster_size[ cluster[i] ];

    gprs_.resize( cluster_size.size() );
    gpr_outputs_.resize( cluster_size.size() );
    LOG("GPR started\n");
    for (int i=0; i<cluster_size.size(); i++)
    {
        gprs_[i].setHyperParameters(options_.gpr_l, options_.gpr_sigma_f, options_.gpr_sigma_n);

        Eigen::MatrixXd cX(X.rows(), cluster_size[i]);
        Eigen::MatrixXd cY(cluster_size[i], Y.rows());

        int c = 0;
        for (int j=0; j<cluster.size(); j++)
        {
            if (cluster[j] == i)
            {
                cX.col(c) = X.col(j);
                cY.row(c) = Y.col(j).transpose();
                c++;
            }
        }

        gprs_[i].setObservationInput(cX);
        gpr_outputs_[i] = cY;
    }
    LOG("GPR finished\n");
}

void LearningMotion::loadTrainedData(const std::string& filename)
{
    FILE* fp = fopen(filename.c_str(), "r");
    if (fp == NULL)
    {
        fprintf(stderr, "error: file doesn't exist [%s]\n", filename.c_str());
        fflush(stderr);
        return;
    }

    char buffer[128];
    while (fscanf(fp, "%s", buffer) == 1)
    {
        LOG("%s\n", buffer);

        if (!strcmp(buffer, "human_model_filename:"))
        {
            fscanf(fp, "%s", buffer);
            human_model_filename_ = buffer;
        }

        else if (!strcmp(buffer, "cluster_centers:"))
        {
            int r, c;
            fscanf(fp, "%*s%d%d", &r, &c);
            cluster_centers_.resize(r);
            for (int i=0; i<r; i++)
            {
                cluster_centers_[i].resize(c);
                for (int j=0; j<c; j++)
                    fscanf(fp, "%lf", &cluster_centers_[i](j));
            }
            fscanf(fp, "%*s");
        }

        else if (!strcmp(buffer, "gprs:"))
        {
            int size;
            fscanf(fp, "%*s%d", &size);
            gprs_.resize(size);

            for (int idx=0; idx<size; idx++)
            {
                double l, f, n;
                fscanf(fp, "%lf%lf%lf", &l, &f, &n);
                gprs_[idx].setHyperParameters(l, f, n);

                int r, c;
                fscanf(fp, "%d%d", &r, &c);
                Eigen::MatrixXd X(r, c);
                for (int i=0; i<r; i++)
                {
                    for (int j=0; j<c; j++)
                        fscanf(fp, "%lf", &X(i,j));
                }

                fscanf(fp, "%d", &r);
                Eigen::MatrixXd kinv(r, r);
                for (int i=0; i<r; i++)
                {
                    for (int j=0; j<r; j++)
                        fscanf(fp, "%lf", &kinv(i,j));
                }

                gprs_[idx].loadTrainedData(X, kinv);
            }
            fscanf(fp, "%*s");
        }

        else if (!strcmp(buffer, "gpr_outputs:"))
        {
            int size;
            fscanf(fp, "%*s%d", &size);
            gpr_outputs_.resize(size);

            for (int idx=0; idx<size; idx++)
            {
                int r, c;
                fscanf(fp, "%d%d", &r, &c);

                Eigen::MatrixXd Y(r, c);
                for (int i=0; i<r; i++)
                {
                    for (int j=0; j<c; j++)
                        fscanf(fp, "%lf", &Y(i,j));
                }

                gpr_outputs_[idx] = Y;
            }
            fscanf(fp, "%*s");
        }
    }

    fclose(fp);
}

void LearningMotion::saveTrainedData(const std::string& filename)
{
    FILE* fp = fopen(filename.c_str(), "w");
    if (fp == NULL)
    {
        fprintf(stderr, "error: file doesn't exist [%s]\n", filename.c_str());
        fflush(stderr);
        return;
    }

    fprintf(fp, "human_model_filename: %s\n", human_model_filename_.c_str());

    fprintf(fp, "cluster_centers: [ %d %d\n", cluster_centers_.size(), cluster_centers_[0].size());
    for (int i=0; i<cluster_centers_.size(); i++)
    {
        for (int j=0; j<cluster_centers_[i].rows(); j++)
            fprintf(fp, "%lf ", cluster_centers_[i](j));
        fprintf(fp, "\n");
    }
    fprintf(fp, "]\n");

    fprintf(fp, "gprs: [ %d\n", gprs_.size());
    for (int i=0; i<gprs_.size(); i++)
    {
        const Eigen::MatrixXd X = gprs_[i].getX();
        const Eigen::MatrixXd kinv = gprs_[i].getKInverse();

        fprintf(fp, "%lf %lf %lf\n", gprs_[i].getL(), gprs_[i].getSigmaF(), gprs_[i].getSigmaN());

        fprintf(fp, "%d %d\n", X.rows(), X.cols());
        for (int i=0; i<X.rows(); i++)
        {
            for (int j=0; j<X.cols(); j++)
                fprintf(fp, "%lf ", X(i,j));
            fprintf(fp, "\n");
        }

        fprintf(fp, "%d\n", kinv.rows());
        for (int i=0; i<kinv.rows(); i++)
        {
            for (int j=0; j<kinv.cols(); j++)
                fprintf(fp, "%lf ", kinv(i,j));
            fprintf(fp, "\n");
        }
    }
    fprintf(fp, "]\n");

    fprintf(fp, "gpr_outputs: [ %d\n", gpr_outputs_.size());
    for (int i=0; i<gpr_outputs_.size(); i++)
    {
        const Eigen::MatrixXd& m = gpr_outputs_[i];
        fprintf(fp, "%d %d\n", m.rows(), m.cols());
        for (int i=0; i<m.rows(); i++)
        {
            for (int j=0; j<m.cols(); j++)
                fprintf(fp, "%lf ", m(i,j));
            fprintf(fp, "\n");
        }
    }
    fprintf(fp, "]\n");

    fclose(fp);
}
