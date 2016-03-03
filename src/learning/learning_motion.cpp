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
    visualizer_ = 0;
    setVisualizerTopic("motion_inference");
}

void LearningMotion::appendColumn(Eigen::MatrixXd &m, const Eigen::VectorXd &x)
{
    m.conservativeResize(Eigen::NoChange, m.cols() + 1);
    m.col( m.cols() - 1 ) = x;
}

void LearningMotion::setOptions(const LearningMotionOptions& options)
{
    options_ = options;
}

void LearningMotion::setHumanModelFilename(const std::string& filename)
{
    human_model_filename_ = filename;
    feature_template_.loadHumanJoints(filename);
}

void LearningMotion::parseData(const char *directory)
{
    num_action_types_ = 4;
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

            if (num_action_types_ < actions(i) + 1)
                num_action_types_ = actions(i) + 1;

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
    input_feature = feature_template_;

    HumanMotionFeature output_feature;
    output_feature = feature_template_;

    const int num_progress_states = 1 << num_action_types_;
    std::vector<Eigen::MatrixXd> pX(num_progress_states, Eigen::MatrixXd(ksi_[0].rows() * options_.input_frames + 8*2*options_.input_frames, 0));
    std::vector<Eigen::MatrixXd> pY(num_progress_states, Eigen::MatrixXd(ksi_[0].rows() * options_.output_frames + 8*2*options_.output_frames, 0));


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

        appendColumn(pX[progress_states_[demo](s)], input_feature.columnFeature());
        appendColumn(pY[progress_states_[demo](s)], output_feature.columnFeature());
    }


    HierarchicalKmeans kmeans;
    kmeans.setVerbose(verbose_);
    kmeans.setSizeLimit( options_.max_cluster_size );
    kmeans.setK( options_.kmeans_k );
    kmeans.setTerminationCondition(100000);

    gprs_.resize( num_progress_states );
    gpr_outputs_.resize( num_progress_states );
    cluster_centers_.resize( num_progress_states );
    current_action_prob_.resize( num_action_types_ );
    means_.resize( num_action_types_ );
    variances_.resize( num_action_types_ );

    for (int p=0; p<pX.size(); p++)
    {
        if (pX[p].cols() > 0)
        {
            LOG("Progress state %x\n", p);

            Eigen::MatrixXd& X = pX[p];
            Eigen::MatrixXd& Y = pY[p];

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

            cluster_centers_[p].resize( cluster_size.size(), Eigen::VectorXd(X.rows()) );
            for (int i=0; i<cluster.size(); i++)
                cluster_centers_[p][ cluster[i] ] += X.col(i) / cluster_size[ cluster[i] ];

            gprs_[p].resize( cluster_size.size() );
            gpr_outputs_[p].resize( cluster_size.size() );
            LOG("GPR started\n");
            for (int i=0; i<cluster_size.size(); i++)
            {
                gprs_[p][i].setHyperParameters(options_.gpr_l, options_.gpr_sigma_f, options_.gpr_sigma_n);

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

                gprs_[p][i].setObservationInput(cX);
                gpr_outputs_[p][i] = cY;
            }
            LOG("GPR finished\n");
        }
    }

    int channel_size = 0;
    for (int p=0; p<gpr_outputs_.size(); p++)
    {
        if (!gpr_outputs_[p].empty())
        {
            channel_size = gpr_outputs_[p][0].cols();
            break;
        }
    }
    for (int a=0; a<num_action_types_; a++)
    {
        means_[a].resize( channel_size );
        variances_[a].resize( channel_size );
    }
}

void LearningMotion::infer(const Eigen::VectorXd &feature, int state, int last_action)
{
    if (cluster_centers_[state].empty())
    {
        //LOG_ERROR("state %d is not defined\n");

        for (int a=0; a<num_action_types_; a++)
        {
            current_action_prob_[a] = 0.0;
        }
    }

    else
    {
        double prob_sum = 0.0;

        for (int a=0; a<num_action_types_; a++)
        {
            current_action_prob_[a] = 0.0;

            const int current_state = state | (1<<a);
            if ((current_state != state || a == last_action) && !cluster_centers_[current_state].empty())
            {
                int cluster_index = 0;
                double min_diff = (feature - cluster_centers_[current_state][0]).squaredNorm();
                for (int i=1; i<cluster_centers_[current_state].size(); i++)
                {
                    const double diff = (feature - cluster_centers_[current_state][i]).squaredNorm();
                    if (min_diff > diff)
                    {
                        min_diff = diff;
                        cluster_index = i;
                    }
                }

                GprMultivariate& gpr = gprs_[current_state][cluster_index];
                const Eigen::MatrixXd& gpr_output = gpr_outputs_[current_state][cluster_index];
                for (int i=0; i<gpr_output.cols(); i++)
                    gpr.regression(gpr_output.col(i), feature, means_[a](i), variances_[a](i));

                current_action_prob_[a] = gpr.getSimilarity(feature);
                prob_sum += current_action_prob_[a];
            }
        }

        if (prob_sum != 0.0)
        {
            for (int a=0; a<num_action_types_; a++)
                current_action_prob_[a] /= prob_sum;
        }

        /*
        printf("state %2d lastaction %2d prob: ", state, last_action);
        for (int a=0; a<num_action_types_; a++)
            printf("%8.6lf ", a, current_action_prob_[a]);
        printf("\n");

        fflush(stdout);
        */
    }
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

        if (!strcmp(buffer, "num_action_types:"))
            fscanf(fp, "%d", num_action_types_);

        else if (!strcmp(buffer, "human_model_filename:"))
        {
            fscanf(fp, "%s", buffer);
            human_model_filename_ = buffer;
        }

        else if (!strcmp(buffer, "cluster_centers:"))
        {
            int num_progress_states;
            fscanf(fp, "%*s%d", &num_progress_states);
            cluster_centers_.resize(num_progress_states);
            for (int p=0; p<num_progress_states; p++)
            {
                int r;
                fscanf(fp, "%d", &r);
                cluster_centers_[p].resize(r);
                for (int i=0; i<r; i++)
                {
                    int c;
                    fscanf(fp, "%d", &c);
                    cluster_centers_[p][i].resize(c);
                    for (int j=0; j<c; j++)
                        fscanf(fp, "%lf", &cluster_centers_[p][i](j));
                }
            }
            fscanf(fp, "%*s");
        }

        else if (!strcmp(buffer, "gprs:"))
        {
            int num_progress_states;
            fscanf(fp, "%*s%d", num_progress_states);
            gprs_.resize(num_progress_states);
            for (int p=0; p<num_progress_states; p++)
            {
                int size;
                fscanf(fp, "%d", &size);
                gprs_[p].resize(size);

                for (int idx=0; idx<size; idx++)
                {
                    double l, f, n;
                    fscanf(fp, "%lf%lf%lf", &l, &f, &n);
                    gprs_[p][idx].setHyperParameters(l, f, n);

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

                    gprs_[p][idx].loadTrainedData(X, kinv);
                }
            }
            fscanf(fp, "%*s");
        }

        else if (!strcmp(buffer, "gpr_outputs:"))
        {
            int num_progress_states;
            fscanf(fp, "%*s%d", &num_progress_states);
            gpr_outputs_.resize(num_progress_states);
            for (int p=0; p<num_progress_states; p++)
            {
                int size;
                fscanf(fp, "%d", &size);
                gpr_outputs_[p].resize(size);

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

                    gpr_outputs_[p][idx] = Y;
                }
            }
            fscanf(fp, "%*s");
        }
    }

    fclose(fp);

    current_action_prob_.resize( num_action_types_ );
    means_.resize( num_action_types_ );
    variances_.resize( num_action_types_ );

    int channel_size = 0;
    for (int p=0; p<gpr_outputs_.size(); p++)
    {
        if (!gpr_outputs_[p].empty())
        {
            channel_size = gpr_outputs_[p][0].cols();
            break;
        }
    }
    for (int a=0; a<num_action_types_; a++)
    {
        means_[a].resize( channel_size );
        variances_[a].resize( channel_size );
    }
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

    fprintf(fp, "num_action_types: %d\n", num_action_types_);

    fprintf(fp, "human_model_filename: %s\n", human_model_filename_.c_str());

    fprintf(fp, "cluster_centers: [ %d\n", cluster_centers_.size());
    for (int p=0; p<cluster_centers_.size(); p++)
    {
        fprintf(fp, "%d\n", cluster_centers_[p].size());
        for (int i=0; i<cluster_centers_[p].size(); i++)
        {
            fprintf(fp, "%d\n", cluster_centers_[p][i].size());
            for (int j=0; j<cluster_centers_[p][i].rows(); j++)
                fprintf(fp, "%lf ", cluster_centers_[p][i](j));
            fprintf(fp, "\n");
        }
    }
    fprintf(fp, "]\n");

    fprintf(fp, "gprs: [ %d\n", gprs_.size());
    for (int p=0; p<gprs_.size(); p++)
    {
        fprintf(fp, "%d\n", gprs_[p].size());
        for (int i=0; i<gprs_[p].size(); i++)
        {
            const Eigen::MatrixXd X = gprs_[p][i].getX();
            const Eigen::MatrixXd kinv = gprs_[p][i].getKInverse();

            fprintf(fp, "%lf %lf %lf\n", gprs_[p][i].getL(), gprs_[p][i].getSigmaF(), gprs_[p][i].getSigmaN());

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
    }
    fprintf(fp, "]\n");

    fprintf(fp, "gpr_outputs: [ %d\n", gpr_outputs_.size());
    for (int p=0; p<gpr_outputs_.size(); p++)
    {
        fprintf(fp, "%d\n", gpr_outputs_[p].size());
        for (int i=0; i<gpr_outputs_[p].size(); i++)
        {
            const Eigen::MatrixXd& m = gpr_outputs_[p][i];
            fprintf(fp, "%d %d\n", m.rows(), m.cols());
            for (int i=0; i<m.rows(); i++)
            {
                for (int j=0; j<m.cols(); j++)
                    fprintf(fp, "%lf ", m(i,j));
                fprintf(fp, "\n");
            }
        }
    }
    fprintf(fp, "]\n");

    fclose(fp);
}

void LearningMotion::setVisualizerTopic(const std::string &topic)
{
    if (visualizer_ != 0)
        delete visualizer_;

    visualizer_ = new MarkerArrayVisualizer(topic.c_str());
}

void LearningMotion::visualizeInferenceResult()
{
    const int num_joints = feature_template_.numJoints();
    const int num_links = feature_template_.numLinks();
    const int cnt = 4;

    const double confidence_level = 0.99;

    int ptr = 0;

    for (int i=0; i<options_.output_frames; i++)
    {
        const std::string ns = "prediction_frame" + std::to_string(i);

        int action = 0;
        double maxprob = 0.;
        for (int a=0; a<num_action_types_; a++)
        {
            if (maxprob < current_action_prob_[a])
            {
                maxprob = current_action_prob_[a];
                action = a;
            }
        }

        std::vector<Eigen::Vector3d> mu(num_joints);
        std::vector<Eigen::Matrix3d> sigma(num_joints);
        std::vector<double> offset(num_joints);

        for (int j=0; j<num_joints; j++)
        {
            mu[j] = means_[action].block(ptr, 0, 3, 1);
            sigma[j] = variances_[action].block(ptr, 0, 3, 1).asDiagonal();
            offset[j] = 0.05;

            ptr += 3;
        }

        for (int j=0; j<num_links; j++)
        {
            const std::pair<int, int> ids = feature_template_.linkIds(j);
            const int i0 = ids.first;
            const int i1 = ids.second;

            Eigen::Vector3d c0 = means_[action].block(ptr - num_joints*3 + 3*i0, 0, 3, 1);
            Eigen::Vector3d v0 = variances_[action].block(ptr - num_joints*3 + 3*i0, 0, 3, 1);
            Eigen::Vector3d c1 = means_[action].block(ptr - num_joints*3 + 3*i1, 0, 3, 1);
            Eigen::Vector3d v1 = variances_[action].block(ptr - num_joints*3 + 3*i1, 0, 3, 1);

            for (int k=1; k<cnt; k++)
            {
                const double t = (double)k / cnt;

                mu.push_back((1-t)*c0 + t*c1);
                sigma.push_back( ((1-t)*(1-t)*v0 + t*t*v1).asDiagonal() );
                offset.push_back(0.09);
            }
        }

        visualizer_->drawGaussianDistributions(ns.c_str(), mu, sigma, confidence_level, offset);
    }

    // classifiers
    const Eigen::Vector3d p[8] =
    {
        Eigen::Vector3d(0.6, 0.3, 0.05),
        Eigen::Vector3d(0.6, 0.1, 0.05),
        Eigen::Vector3d(0.6, -0.1, 0.05),
        Eigen::Vector3d(0.6, -0.3, 0.05),
        Eigen::Vector3d(0.4, 0.3, 0.05),
        Eigen::Vector3d(0.4, 0.1, 0.05),
        Eigen::Vector3d(0.4, -0.1, 0.05),
        Eigen::Vector3d(0.4, -0.3, 0.05)
    };
    std::vector<Eigen::Vector3d> positions;

    positions.push_back(p[0]);
    positions.push_back(p[1]);
    positions.push_back(p[2]);
    positions.push_back(p[3]);
    positions.push_back(p[4]);
    positions.push_back(p[5]);
    positions.push_back(p[6]);
    positions.push_back(p[7]);

    std::vector<std_msgs::ColorRGBA> colors;
    for (int a=0; a<8; a++)
    {
        //if (a==0) continue;
        //if (a==1) continue;
        //if (a==2) continue;
        //if (a==3) continue;
        //if (a==4) continue;
        //if (a==5) continue;
        //if (a==6) continue;
        //if (a==7) continue;

        std_msgs::ColorRGBA color;
        color.r = 1-current_action_prob_[a];
        color.g = 1-current_action_prob_[a];
        color.b = 1-current_action_prob_[a];
        color.a = 1;
        colors.push_back(color);
    }

    visualizer_->drawCubes("positions", Eigen::Vector3d(0.1, 0.1, 0.1), positions, colors);
}

void LearningMotion::saveFutureMotion(const std::string& filename)
{
    const int num_joints = feature_template_.numJoints();
    const int num_links = feature_template_.numLinks();
    const int cnt = 4;

    FILE* fp = fopen(filename.c_str(), "a");

    fprintf(fp, "%d\n", options_.output_frames);

    int ptr = 0;

    for (int i=0; i<options_.output_frames; i++)
    {
        std::vector<Eigen::Vector3d> mu;
        std::vector<Eigen::Matrix3d> sigma;
        std::vector<double> offset;
        std::vector<double> weight;

        for (int action=0; action<num_action_types_; action++)
        {
            for (int j=0; j<num_joints; j++)
            {
                mu.push_back( means_[action].block(ptr, 0, 3, 1) );
                sigma.push_back( variances_[action].block(ptr, 0, 3, 1).asDiagonal() );
                offset.push_back( 0.09 );
                weight.push_back( current_action_prob_[action] );

                ptr += 3;
            }

            for (int j=0; j<num_links; j++)
            {
                const std::pair<int, int> ids = feature_template_.linkIds(j);
                const int i0 = ids.first;
                const int i1 = ids.second;

                Eigen::Vector3d c0 = means_[action].block(ptr - num_joints*3 + 3*i0, 0, 3, 1);
                Eigen::Vector3d v0 = variances_[action].block(ptr - num_joints*3 + 3*i0, 0, 3, 1);
                Eigen::Vector3d c1 = means_[action].block(ptr - num_joints*3 + 3*i1, 0, 3, 1);
                Eigen::Vector3d v1 = variances_[action].block(ptr - num_joints*3 + 3*i1, 0, 3, 1);

                for (int k=1; k<cnt; k++)
                {
                    const double t = (double)k / cnt;

                    mu.push_back((1-t)*c0 + t*c1);
                    sigma.push_back( ((1-t)*(1-t)*v0 + t*t*v1).asDiagonal() );
                    offset.push_back(0.09);
                    weight.push_back( current_action_prob_[action] );
                }
            }
        }

        fprintf(fp, "%d\n", mu.size());
        for (int i=0; i<mu.size(); i++)
        {
            for (int j=0; j<3; j++) fprintf(fp, "%lf ", mu[i](j));
            for (int j=0; j<3; j++)
                for (int k=0; k<3; k++) fprintf(fp, "%lf ", sigma[i](j,k));
            fprintf(fp, "%lf %lf\n", offset[i], weight[i]);
        }
    }

    fclose(fp);
}
