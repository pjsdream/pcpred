#include <pcpred/learning/motion_predictor.h>

#include <pcpred/learning/hierarchical_kmeans.h>

using namespace pcpred;


MotionPredictor::MotionPredictor()
{
    action_availability_ = 0;

    remaining_time_ = 1. / fps_;

    visualizer_ = 0;
    setVisualizerTopic("motion_inference");
}

void MotionPredictor::setVisualizerTopic(const std::string& topic)
{
    if (visualizer_ != 0)
        delete visualizer_;

    visualizer_ = new MarkerArrayVisualizer(topic.c_str());
}

void MotionPredictor::appendColumn(Eigen::MatrixXd& matrix, const Eigen::VectorXd& column)
{
    matrix.conservativeResize(Eigen::NoChange, matrix.cols() + 1);
    matrix.col( matrix.cols() - 1 ) = column;
}

void MotionPredictor::initialize(const std::string& data_directory, int benchmark_number)
{
    data_directory_ = data_directory;
    benchmark_number_ = benchmark_number;

    remaining_time_ = 1. / fps_;

    parseData();

    learn();
    initializeActionOrder();
    initializeIdleMotion();
}

void MotionPredictor::setActionAvailability(int action, bool availability)
{
    action_availability_ |= (int)availability << action;
}


void MotionPredictor::moveToFuture(double time)
{
    while (remaining_time_ < time)
    {
        time -= remaining_time_;
        remaining_time_ = 1. / fps_;

        if (current_ksi_column_ >= current_ksi_.cols())
        {
            int next_action = -1;
            if (action_order_index_ < action_order_.size())
            {
                int next_desired_action = action_order_[action_order_index_];
                if (action_availability_ & (1 << next_desired_action))
                {
                    next_action = next_desired_action;
                    action_order_index_ ++;
                }
            }

            if (next_action == -1)
                appendIdle();

            else
            {
                appendIdle();
                appendIdle();
                appendIdle();
                appendIdle();
                appendIdle();
                appendAction(next_action);
            }
        }

        current_feature_.addFrame( current_ksi_.col( current_ksi_column_ ) );
        current_feature_.retainLastFrames(options_.input_frames);

        if (current_actions_[current_ksi_column_] != -1)
        {
            state_ |= (1 << current_actions_[current_ksi_column_]);
            last_action_ = current_actions_[current_ksi_column_];
        }

        current_ksi_column_++;
    }
    remaining_time_ -= time;

    predictFutureMotion();
}

void MotionPredictor::getPredictedEllipsoids(double time, std::vector<Eigen::Vector3d>& mu, std::vector<Eigen::Matrix3d>& sigma, std::vector<double>& offsets, std::vector<double>& weights)
{
    mu.clear();
    sigma.clear();
    offsets.clear();
    weights.clear();

    const int num_joints = feature_template_.numJoints();
    const int num_links = feature_template_.numLinks();
    const int cnt = 4;

    for (int a=0; a<num_action_types_; a++)
    {
        int ptr = 0;
        int i = time / (1. / fps_);
        if (i >= options_.output_frames)
            i = options_.output_frames - 1;

        for (int j=0; j<num_joints; j++)
        {
            mu.push_back( means_[a].block(ptr, 0, 3, 1) );
            sigma.push_back( variances_[a].block(ptr, 0, 3, 1).asDiagonal() );
            offsets.push_back( 0.05 + (double)i / (options_.output_frames - 1) * 0.01 );
            weights.push_back( current_action_prob_[a] );

            ptr += 3;
        }

        for (int j=0; j<num_links; j++)
        {
            const std::pair<int, int> ids = feature_template_.linkIds(j);
            const int i0 = ids.first;
            const int i1 = ids.second;

            Eigen::Vector3d c0 = means_[a].block(ptr - num_joints*3 + 3*i0, 0, 3, 1);
            Eigen::Vector3d v0 = variances_[a].block(ptr - num_joints*3 + 3*i0, 0, 3, 1);
            Eigen::Vector3d c1 = means_[a].block(ptr - num_joints*3 + 3*i1, 0, 3, 1);
            Eigen::Vector3d v1 = variances_[a].block(ptr - num_joints*3 + 3*i1, 0, 3, 1);

            for (int k=1; k<cnt; k++)
            {
                const double t = (double)k / cnt;

                mu.push_back((1-t)*c0 + t*c1);
                sigma.push_back( ((1-t)*(1-t)*v0 + t*t*v1).asDiagonal() );
                offsets.push_back(0.05 + (double)i / (options_.output_frames - 1) * 0.01);
                weights.push_back( current_action_prob_[a] );
            }
        }
    }
}

int MotionPredictor::getNextRobotAction()
{
    /*
    int max_index = -1;
    for (int i=0; i<num_action_types_; i++)
    {
        if (next_action_histogram_[state_][i] != 0.0 && (action_availability_ & (1 << i)) == 0 &&
                (max_index == -1 || next_action_histogram_[state_][i] != 0.0))
            max_index = i;
    }
    if (max_index != -1)
        return max_index;

    std::vector<int> candidates;
    for (int i=0; i<num_action_types_; i++)
    {
        if ((action_availability_ & (1 << i)) == 0)
            candidates.push_back(i);
    }
    if (candidates.empty())
        return -1;

    return candidates[ rand() % candidates.size() ];
    */

    static int next_action_index = 0;
    if (next_action_index == action_order_.size())
        return -1;

    return action_order_[next_action_index++];
}


void MotionPredictor::visualizeFutureMotion()
{
    const int num_joints = feature_template_.numJoints();
    const int num_links = feature_template_.numLinks();
    const int cnt = 4;

    const double confidence_level = 0.95;

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
            offset[j] = 0.05 + (double)i / (options_.output_frames) * 0.01;

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
                offset.push_back(0.05 + (double)i / (options_.output_frames) * 0.01);
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

    // current action probs
    std::vector<std_msgs::ColorRGBA> colors;
    for (int a=0; a<num_action_types_; a++)
    {
        const std::string ns = "target_" + std::to_string(a);

        positions.clear();
        colors.clear();

        positions.push_back(p[a]);

        std_msgs::ColorRGBA color;
        color.r = 1-current_action_prob_[a];
        color.g = 1-current_action_prob_[a];
        color.b = 1-current_action_prob_[a];
        color.a = 0.5;
        colors.push_back(color);

        visualizer_->drawCubes(ns.c_str(), Eigen::Vector3d(0.1, 0.1, 0.1), positions, colors);
    }

    // desired probs
    positions.clear();
    colors.clear();
    for (int a=0; a<num_action_types_; a++)
    {
        const std::string ns = "desire_" + std::to_string(a);

        positions.clear();
        colors.clear();

        positions.push_back(p[a]);

        std_msgs::ColorRGBA color;
        color.r = 1-desired_action_prob_[a];
        color.g = 1-desired_action_prob_[a];
        color.b = 1-desired_action_prob_[a];
        color.a = 0.5;
        colors.push_back(color);

        visualizer_->drawCubes(ns.c_str(), Eigen::Vector3d(0.1, 0.1, 0.1), positions, colors);
    }
}

void MotionPredictor::visualizeHuman()
{
    const int num_joints = feature_template_.numJoints();
    const int num_links = feature_template_.numLinks();
    const int cnt = 4;

    int ptr = 0;

    const int i = 0;
    {
        const std::string ns = "human";

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

        for (int j=0; j<num_joints; j++)
        {
            mu[j] = means_[action].block(ptr, 0, 3, 1);
            ptr += 3;
        }

        for (int j=0; j<num_links; j++)
        {
            const std::pair<int, int> ids = feature_template_.linkIds(j);
            const int i0 = ids.first;
            const int i1 = ids.second;

            Eigen::Vector3d c0 = means_[action].block(ptr - num_joints*3 + 3*i0, 0, 3, 1);
            Eigen::Vector3d c1 = means_[action].block(ptr - num_joints*3 + 3*i1, 0, 3, 1);

            for (int k=1; k<cnt; k++)
            {
                const double t = (double)k / cnt;
                mu.push_back((1-t)*c0 + t*c1);
            }
        }

        visualizer_->drawSphereList(ns.c_str(), mu, 0.05);
    }
}


void MotionPredictor::initializeActionOrder()
{
    action_order_index_ = 0;
    state_ = 0;

    for (int i=0; i<num_action_types_; i++)
        action_order_.push_back(i);

    /*
    action_order_.push_back(1);
    action_order_.push_back(3);
    action_order_.push_back(2);
    action_order_.push_back(0);
    */

    /*
    const int episode = rand() % num_episodes_;
    for (int i=0; i<actions_[episode].rows(); i++)
    {
        if (action_order_.empty() || (i && actions_[episode](i) != action_order_.back()))
            action_order_.push_back(actions_[episode](i));
    }
    */
}

void MotionPredictor::initializeIdleMotion()
{
    current_feature_ = feature_template_;
    current_ksi_column_ = 0;

    // 2 seconds idle
    appendIdle();
    appendIdle();

    // read columns as much as input frames
    for (int i=0; i<options_.input_frames; i++)
        current_feature_.addFrame(current_ksi_.col(i));
    last_action_ = -1;
    current_ksi_column_ = options_.input_frames;
}

void MotionPredictor::parseData()
{
    // parse human data
    char filename[128];
    sprintf(filename, "%s/human_upper_body.txt", data_directory_.c_str());
    feature_template_.loadHumanJoints(filename);

    // parse learning options
    sprintf(filename, "%s/learning_options.txt", data_directory_.c_str());
    options_.parse(filename);

    // parse training data
    int e = 0;
    while (true)
    {
        char filename[128];
        sprintf(filename, "%s/B%02d/sequence%03d.txt", data_directory_.c_str(), benchmark_number_, e);

        FILE* fp = fopen(filename, "r");
        if (fp == NULL)
            break;

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
    num_episodes_ = e;

    // parse actions for motion generation
    num_sequences_.resize(num_action_types_);
    for (int i=0; i<num_action_types_; i++)
    {
        int n = 0;
        while (true)
        {
            char filename[128];
            sprintf(filename, "%s/J%02d/sequence%03d.txt", data_directory_.c_str(), i, n);
            FILE* fp = fopen(filename, "r");
            if (fp == NULL)
            {
                num_sequences_[i] = n;
                break;
            }
            fclose(fp);

            n++;
        }
    }
}

void MotionPredictor::learn()
{
    HumanMotionFeature input_feature;
    input_feature = feature_template_;

    HumanMotionFeature output_feature;
    output_feature = feature_template_;

    const int num_progress_states = 1 << num_action_types_;
    std::vector<Eigen::MatrixXd> pX(num_progress_states, Eigen::MatrixXd(ksi_[0].rows() * options_.input_frames + 8*2*options_.input_frames, 0));
    std::vector<Eigen::MatrixXd> pY(num_progress_states, Eigen::MatrixXd(ksi_[0].rows() * options_.output_frames + 8*2*options_.output_frames, 0));


    // next action sequence learning
    next_action_histogram_.resize(num_progress_states);
    for (int i=0; i<num_progress_states; i++)
        next_action_histogram_[i].resize(num_action_types_);

    for (int i=0; i<actions_.size(); i++)
    {
        next_action_histogram_[ 0 ][ actions_[i][0] ]++;
        for (int j=0; j<actions_[i].size() - 1; j++)
        {
            if (!(progress_states_[i][j] & (1 << actions_[i][j + 1])))
                next_action_histogram_[ progress_states_[i][j] ][ actions_[i][j + 1] ]++;
        }
    }
    for (int i=0; i<num_progress_states; i++)
    {
        double prob_sum = 0.0;
        for (int j=0; j<num_action_types_; j++)
            prob_sum += next_action_histogram_[i][j];

        if (prob_sum != 0.0)
        {
            for (int j=0; j<num_action_types_; j++)
                next_action_histogram_[i][j] /= prob_sum;
        }
    }


    // data sampling from episodes
    int num_possibilities = 0;
    for (int i=0; i<ksi_.size(); i++)
        num_possibilities += ksi_[i].cols() - options_.input_frames - options_.output_frames;

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


    // clustering
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

    // gpr learning
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


void MotionPredictor::predictFutureMotion()
{
    double prob_sum = 0.0;
    double desired_prob_sum = 0.0;

    const Eigen::VectorXd feature = current_feature_.columnFeature();

    /*
    for (int a=0; a<num_action_types_; a++)
    {
        current_action_prob_[a] = 0.0;

        const int current_state = state_ | (1<<a);
        if ((current_state != state_ || a == last_action_) && !cluster_centers_[current_state].empty())
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
    */

    Eigen::MatrixXd old_current_ksi = current_ksi_;
    Eigen::VectorXi old_current_actions = current_actions_;
    while (current_ksi_column_ + options_.output_frames > current_ksi_.cols())
        appendIdle();

    desired_action_prob_.resize(num_action_types_);

    for (int a=0; a<num_action_types_; a++)
    {
        current_action_prob_[a] = 0.0;

        Eigen::MatrixXd means = current_ksi_.block(0, current_ksi_column_, 27, options_.output_frames);

        means_[a] = Eigen::Map<Eigen::VectorXd>(means.data(), means.rows() * means.cols());
        means_[a].conservativeResize(645);

        variances_[a].conservativeResize( means_[a].size() );
        for (int i=0; i<means_[a].size(); i++)
            variances_[a](i) = 0.00001;

        current_action_prob_[a] = 1.0;
        prob_sum += current_action_prob_[a];

        desired_action_prob_[a] = next_action_histogram_[action_availability_][a];
        desired_prob_sum += desired_action_prob_[a];
    }
    current_ksi_ = old_current_ksi;
    current_actions_ = old_current_actions;

    // normalize probability
    if (prob_sum != 0.0)
    {
        for (int a=0; a<num_action_types_; a++)
            current_action_prob_[a] /= prob_sum;
    }
    if (desired_prob_sum != 0.0)
    {
        for (int a=0; a<num_action_types_; a++)
            desired_action_prob_[a] /= desired_prob_sum;
    }

    /*
    printf("state %2d lastaction %2d prob: ", state, last_action);
    for (int a=0; a<num_action_types_; a++)
        printf("%8.6lf ", a, current_action_prob_[a]);
    printf("\n");

    fflush(stdout);
    */
}


void MotionPredictor::appendIdle()
{
    const int action = 0; // rand() % num_action_types_;
    const int sequence = 0; // rand() % num_sequences_[action];

    appendActionLastFrames(action, sequence, fps_, true);
}

void MotionPredictor::appendAction(int action)
{
    const int sequence = rand() % num_sequences_[action];
    appendActionSequence(action, sequence);
}

void MotionPredictor::appendActionSequence(int action, int sequence)
{
    appendActionLastFrames(action, sequence);
}

void MotionPredictor::appendActionLastFrames(int action, int sequence, int last_frames, bool is_idle)
{
    char sequence_filename[128];
    sprintf(sequence_filename, "%s/J%02d/sequence%03d.txt", data_directory_.c_str(), action, sequence);

    FILE* ifp = fopen(sequence_filename, "r");
    int r, c;
    fscanf(ifp, "%d%d", &r, &c);
    Eigen::MatrixXd motion(r, c);
    for (int i=0; i<r; i++)
    {
        for (int j=0; j<c; j++)
            fscanf(ifp, "%lf", &motion(i,j));
    }

    if (last_frames == -1)
        last_frames = c;

    current_ksi_.conservativeResize(r, current_ksi_.cols() + last_frames);
    current_ksi_.block(0,  current_ksi_.cols() - last_frames, r, last_frames) = motion.block( 0, motion.cols() - last_frames, motion.rows(), last_frames);
    blendMotion(current_ksi_, current_ksi_.cols() - last_frames, motion_blend_frames_);

    current_actions_.conservativeResize(current_actions_.rows() + last_frames);
    for (int i=c - last_frames; i<c; i++)
        current_actions_( current_actions_.rows()-c+i ) = is_idle ? -1 : action;
}

void MotionPredictor::blendMotion(Eigen::MatrixXd& motion, int frame, int motion_blend_frames)
{
    const int s = frame - motion_blend_frames + 1;
    const int e = frame + motion_blend_frames;
    if (s < 0 || e >= motion.cols())
        return;

    const double alpha = 0.5;

    Eigen::MatrixXd block(motion.rows(), motion_blend_frames*2);

    for (int i=0; i<motion_blend_frames * 2; i++)
    {
        const double t = (double)i / motion_blend_frames / 2.;
        block.col(i) = (1-alpha) * ((1-t) * motion.col(s) + t * motion.col(e)) + alpha * motion.col(s+i);
    }

    motion.block(0, s, motion.rows(), motion_blend_frames*2) = block;
}
