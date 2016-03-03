#include <pcpred/learning/future_motion.h>

#include <resource_retriever/retriever.h>

#include <stdio.h>

using namespace pcpred;


FutureMotion::FutureMotion()
{
    visualizer_ = 0;
    setVisualizerTopic("future_motion");
}

void FutureMotion::setVisualizerTopic(const std::string& topic)
{
    if (visualizer_ != 0)
        delete visualizer_;

    visualizer_ = new MarkerArrayVisualizer(topic.c_str());
}

void FutureMotion::loadSavedData(const std::string& filename)
{
    resource_retriever::Retriever r;
    resource_retriever::MemoryResource resource;

    try
    {
        resource = r.get( filename.c_str() );
    }
    catch (resource_retriever::Exception& e)
    {
        fprintf(stderr, "Failed to retrieve file: %s", e.what());
        fflush(stderr);
        return;
    }

    FILE* fp = fopen("out.txt", "w");
    fwrite(resource.data.get(), resource.size, 1, fp);
    fclose(fp);

    fp = fopen("out.txt", "r");

    fps_ = 15;

    for (int i=0; ; i++)
    {
        bool end = false;

        if (fscanf(fp, "%d", &future_frames_) != 1)
            break;

        data_.push_back( std::vector<std::vector<Data> >(future_frames_) );
        for (int j=0; j<future_frames_; j++)
        {
            fscanf(fp, "%d", &n_);

            data_[i][j].resize(n_);

            for (int k=0; k<n_; k++)
            {
                Eigen::Vector3d c;
                Eigen::Matrix3d s;
                double o;
                double w;

                for (int i=0; i<3; i++)
                {
                    if (fscanf(fp, "%lf", &c(i)) != 1)
                    {
                        end = true;
                        break;
                    }
                }
                if (end)
                    break;

                for (int i=0; i<3; i++)
                    for (int j=0; j<3; j++)
                        fscanf(fp, "%lf", &s(i,j));
                fscanf(fp, "%lf%lf", &o, &w);

                data_[i][j][k] = Data(c, s, o, w);
            }
            if (end)
                break;
        }
        if (end)
        {
            data_.pop_back();
            break;
        }
    }

    fclose(fp);

    frames_ = data_.size();
}

void FutureMotion::getGaussianDistributions(double current_time, double future_time, std::vector<Eigen::Vector3d>& mu, std::vector<Eigen::Matrix3d>& sigma, std::vector<double>& offset, std::vector<double>& weight)
{
    int cidx = current_time / fps_;
    if (cidx == frames_) cidx--;
    const double t = current_time - cidx;

    int fidx = future_time / fps_;
    if (fidx == future_frames_) fidx--;
    const double s = future_time - fidx;

    mu.resize(n_);
    sigma.resize(n_);
    offset.resize(n_);
    weight.resize(n_);

    for (int i=0; i<n_; i++)
    {
        mu[i]     = (1-t) * (1-s) * data_[cidx][fidx][i].mu     + (1-t) * s * data_[cidx][fidx+1][i].mu     + t * (1-s) * data_[cidx+1][fidx][i].mu     + t * s * data_[cidx+1][fidx+1][i].mu;
        sigma[i]  = (1-t) * (1-s) * data_[cidx][fidx][i].sigma  + (1-t) * s * data_[cidx][fidx+1][i].sigma  + t * (1-s) * data_[cidx+1][fidx][i].sigma  + t * s * data_[cidx+1][fidx+1][i].sigma;
        offset[i] = (1-t) * (1-s) * data_[cidx][fidx][i].offset + (1-t) * s * data_[cidx][fidx+1][i].offset + t * (1-s) * data_[cidx+1][fidx][i].offset + t * s * data_[cidx+1][fidx+1][i].offset;
        weight[i] = (1-t) * (1-s) * data_[cidx][fidx][i].weight + (1-t) * s * data_[cidx][fidx+1][i].weight + t * (1-s) * data_[cidx+1][fidx][i].weight + t * s * data_[cidx+1][fidx+1][i].weight;
    }
}

void FutureMotion::visualizePrediction(const std::string& ns, double current_time, double future_time, double probability)
{
    std::vector<Eigen::Vector3d> mu;
    std::vector<Eigen::Matrix3d> sigma;
    std::vector<double> offset;
    std::vector<double> weight;

    getGaussianDistributions(current_time, future_time, mu, sigma, offset, weight);

    visualizer_->drawGaussianDistributions(ns.c_str(), mu, sigma, 0.95, offset);
}

void FutureMotion::visualizeHuman()
{
}
