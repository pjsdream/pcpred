#include <pcpred/learning/learning_motion.h>

#include <pcpred/learning/hierarchical_kmeans.h>

#include <stdio.h>
#include <string.h>

#include <cstdlib>

using namespace pcpred;


LearningMotionOptions::LearningMotionOptions()
{
    input_duration = 1.0;
    input_num_pieces = 5;
    output_duration = 1.0;
    output_num_pieces = 5;
    num_samples = 100000;
    max_cluster_size = 100;
    kmeans_k = 2;
}

void LearningMotionOptions::parse(const char* filename)
{
    FILE* fp = fopen(filename, "r");
    if (fp == NULL)
        return;

    char param[128];
    while (fscanf(fp, "%s", param) == 1)
    {
        if (!strcmp(param, "input_duration:"))
            fscanf(fp, "%lf", &input_duration);

        else if (!strcmp(param, "output_duration:"))
            fscanf(fp, "%lf", &output_duration);

        else if (!strcmp(param, "input_num_pieces:"))
            fscanf(fp, "%d", &input_num_pieces);

        else if (!strcmp(param, "output_num_pieces:"))
            fscanf(fp, "%d", &output_num_pieces);

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
    printf("duration         = %lfs(input), %lfs(output)\n", input_duration, output_duration);
    printf("# curve pieces   = %d(input)  %d(output)\n", input_num_pieces, output_num_pieces);
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

void LearningMotion::parseData(const char *filename)
{
    FILE* fp = fopen(filename, "r");
    if (fp == NULL)
        return;

    char name[128];
    double t, x, y, z;
    while (fscanf(fp, "%s%lf%lf%lf%lf", name, &t, &x, &y, &z) == 5)
    {
        Stream s;
        s.joint_name = name;
        s.t = t;
        s.position = Eigen::Vector3d(x, y, z);
        stream_.push_back(s);
    }

    fclose(fp);
}

void LearningMotion::learn()
{
    /*
    const HumanMotionFeature::FeatureType feature_type = HumanMotionFeature::FEATURE_TYPE_ABSOLUTE_POSITION;
    const double last_time = stream_.rbegin()->t;

    HumanMotionFeature input_feature;
    input_feature.setCurveShape(options_.input_num_pieces, options_.input_duration);

    HumanMotionFeature output_feature;
    output_feature.setCurveShape(options_.output_num_pieces, options_.output_duration);

    Eigen::MatrixXd X(1, options_.num_samples);
    Eigen::MatrixXd input_encoding(1, options_.num_samples);
    Eigen::MatrixXd output_encoding(1, options_.num_samples);


    int input_feature_feature_size;
    int input_feature_encoding_size;
    int output_feature_encoding_size;

    // data sampling from video
    for (int i=0; i<options_.num_samples; i++)
    {
        input_feature.clear();
        output_feature.clear();

        int s;
        do
        {
            s = rand() % stream_.size();
        } while(stream_[s].t > last_time - options_.input_duration + options_.output_duration);

        int p = s;
        while (p < stream_.size() && stream_[p].t <= stream_[s].t + options_.input_duration)
        {
            input_feature.observe(stream_[p].joint_name, stream_[p].t, stream_[p].position);
            p++;
        }
        int s_output = p;
        while (p < stream_.size() && stream_[p].t <= stream_[s_output].t + options_.output_duration)
        {
            output_feature.observe(stream_[p].joint_name, stream_[p].t, stream_[p].position);
            p++;
        }

        // size is available after the first sampling
        if (i==0)
        {
            input_feature_feature_size = input_feature.featureSize(feature_type);
            input_feature_encoding_size = input_feature.encodingSize();
            output_feature_encoding_size = output_feature.encodingSize();

            X.conservativeResize(input_feature_feature_size, Eigen::NoChange);
            input_encoding.conservativeResize(input_feature_encoding_size, Eigen::NoChange);
            output_encoding.conservativeResize(output_feature_encoding_size, Eigen::NoChange);

            if (verbose_)
            {
                printf("input  feature dimension   = %d\n", input_feature_feature_size);
                printf("input  curve encoding size = %d\n", input_feature_encoding_size);
                printf("output curve encoding size = %d\n", output_feature_encoding_size);
                fflush(stdout);
            }
        }

        X.col(i) = input_feature.toFeature( feature_type );
        input_encoding.col(i) = input_feature.encode();
        output_encoding.col(i) = output_feature.encode();
    }

    if (verbose_)
    {
        HierarchicalKmeans kmeans;
        kmeans.setVerbose(verbose_);
        kmeans.setSizeLimit( options_.max_cluster_size );
        kmeans.setK( options_.kmeans_k );
        kmeans.setTerminationCondition(100000);

        printf("starting hierarchical K-means clustering\n");
        kmeans.clusterSizeConstraint(X);
    }
    */
}
