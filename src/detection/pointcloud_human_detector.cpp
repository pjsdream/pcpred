#include <pcpred/detection/pointcloud_human_detector.h>

#include <stdlib.h>

using namespace pcpred;


PointcloudHumanDetector::PointcloudHumanDetector()
{
    visualizer_ = 0;

    setSphereRadius(0.05);
    setNumberSpheres(100);

    srand(1234);
}

void PointcloudHumanDetector::observe(const Pointcloud& pointcloud)
{
    const int num_points = pointcloud.size();

    sphere_centers_.resize( num_spheres_ );

    for (int i=0; i<num_spheres_; i++)
    {
        const int point_index = rand() % num_points;
        const Eigen::Vector3d point = pointcloud.point(point_index);

        sphere_centers_[i] = point;
    }
}


void PointcloudHumanDetector::setVisualizerTopic(const char* topic)
{
    if (visualizer_ != 0)
        delete visualizer_;

    visualizer_ = new MarkerArrayVisualizer(topic);
}

void PointcloudHumanDetector::visualizeHuman()
{
    visualizer_->drawSphereList("human", sphere_centers_, sphere_radius_);
}
