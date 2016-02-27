#include <pcpred/feature/human_motion_feature.h>

#include <utility>

#include <stdio.h>

using namespace pcpred;


HumanMotionFeature::HumanMotionFeature()
{
    setVisualizerTopic("human_motion");
}

void HumanMotionFeature::loadHumanJoints(const std::string& filename)
{
    clear();

    FILE* fp = fopen(filename.c_str(), "r");

    char name[128];

    int num_joints;
    fscanf(fp, "%d", &num_joints);
    for (int i=0; i<num_joints; i++)
    {
        fscanf(fp, "%s", name);
        joint_names_.push_back(name);
        map_joint_to_index_[name] = i;
    }

    int num_links;
    fscanf(fp, "%d", &num_links);
    for (int i=0; i<num_links; i++)
    {
        fscanf(fp, "%s", name);
        const int i0 = map_joint_to_index_[name];
        fscanf(fp, "%s", name);
        const int i1 = map_joint_to_index_[name];

        links_.push_back(std::make_pair(i0, i1));
    }

    fclose(fp);

    feature_.conservativeResize(joint_names_.size() * 3, 0);
}

void HumanMotionFeature::clear()
{
    joint_names_.clear();
    map_joint_to_index_.clear();
    links_.clear();
}

void HumanMotionFeature::clearFeature()
{
    feature_.conservativeResize(Eigen::NoChange, 0);
}

void HumanMotionFeature::loadFeature(const std::string &filename)
{
    FILE* fp = fopen(filename.c_str(), "r");

    int r, c;
    fscanf(fp, "%d%d", &r, &c);
    feature_.resize(r, c);

    for (int i=0; i<r; i++)
    {
        for (int j=0; j<c; j++)
            fscanf(fp, "%lf", &feature_(i,j));
    }

    fclose(fp);
}

void HumanMotionFeature::saveFeature(const std::string &filename)
{
    FILE* fp = fopen(filename.c_str(), "w");

    fprintf(fp, "%d %d\n", feature_.rows(), feature_.cols());

    for (int i=0; i<feature_.rows(); i++)
    {
        for (int j=0; j<feature_.cols(); j++)
            fprintf(fp, "%lf ", feature_(i,j));
        fprintf(fp, "\n");
    }

    fclose(fp);
}

Eigen::MatrixXd HumanMotionFeature::feature()
{
    return feature_;
}

Eigen::VectorXd HumanMotionFeature::columnFeature()
{
    return Eigen::Map<Eigen::VectorXd>(feature_.data(), feature_.cols() * feature_.rows());
}

int HumanMotionFeature::columnFeatureSize()
{
    return feature_.cols() * feature_.rows();
}

int HumanMotionFeature::frameSize()
{
    return joint_names_.size() * 3;
}

void HumanMotionFeature::addFrame(const Eigen::VectorXd& column)
{
    feature_.conservativeResize(Eigen::NoChange, feature_.cols() + 1);
    feature_.col( feature_.cols() - 1 ) = column;
}

void HumanMotionFeature::setVisualizerTopic(const std::string& topic)
{
    ros::NodeHandle nh;
    publisher_.shutdown();
    publisher_ = nh.advertise<visualization_msgs::MarkerArray>(topic, 100);
    ros::Duration(1.0).sleep();
}

void HumanMotionFeature::visualizeHumanMotion(const std::string& ns)
{
    if (feature_.cols() > 0)
        visualizeHumanMotion(0, feature_.cols(), ns);
}

void HumanMotionFeature::visualizeHumanMotion(int start, int end, const std::string& ns)
{
    const double r = 0.025;
    const double l = 0.01;

    std_msgs::ColorRGBA color0;
    color0.r = 1;
    color0.g = 1;
    color0.b = 1;
    color0.a = 1;

    std_msgs::ColorRGBA color1;
    color1.r = 0;
    color1.g = 0;
    color1.b = 0;
    color1.a = 1;

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = ns;
    marker.id = 0;
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;
    marker.pose.orientation.w = 1.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;

    // joints
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.scale.x = r * 2.;
    marker.scale.y = r * 2.;
    marker.scale.z = r * 2.;
    for (int i=start; i<end; i++)
    {
        const double t = (double)(i-start) / (end-start-1);
        std_msgs::ColorRGBA color;
        color.r = (1-t) * color0.r + t * color1.r;
        color.g = (1-t) * color0.g + t * color1.g;
        color.b = (1-t) * color0.b + t * color1.b;
        color.a = (1-t) * color0.a + t * color1.a;

        for (int j=0; j<joint_names_.size(); j++)
        {
            geometry_msgs::Point point;
            point.x = feature_(3*j+0, i);
            point.y = feature_(3*j+1, i);
            point.z = feature_(3*j+2, i);
            marker.points.push_back(point);
            marker.colors.push_back(color);
        }
    }
    marker_array.markers.push_back(marker);

    // links
    marker.colors.clear();
    marker.points.clear();
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.id = 1;
    marker.scale.x = l;
    for (int i=start; i<end; i++)
    {
        const double t = (double)(i-start) / (end-start-1);
        std_msgs::ColorRGBA color;
        color.r = (1-t) * color0.r + t * color1.r;
        color.g = (1-t) * color0.g + t * color1.g;
        color.b = (1-t) * color0.b + t * color1.b;
        color.a = (1-t) * color0.a + t * color1.a;

        for (int j=0; j<links_.size(); j++)
        {
            const int j0 = links_[j].first;
            const int j1 = links_[j].second;

            geometry_msgs::Point point;
            point.x = feature_(3*j0+0, i);
            point.y = feature_(3*j0+1, i);
            point.z = feature_(3*j0+2, i);
            marker.points.push_back(point);
            marker.colors.push_back(color);

            point.x = feature_(3*j1+0, i);
            point.y = feature_(3*j1+1, i);
            point.z = feature_(3*j1+2, i);
            marker.points.push_back(point);
            marker.colors.push_back(color);
        }
    }
    marker_array.markers.push_back(marker);

    // temporal links
    marker.colors.clear();
    marker.points.clear();
    marker.id = 2;
    for (int i=start; i<end-1; i++)
    {
        const double t0 = (double)(i-start) / (end-start-1);
        std_msgs::ColorRGBA colort0;
        colort0.r = (1-t0) * color0.r + t0 * color1.r;
        colort0.g = (1-t0) * color0.g + t0 * color1.g;
        colort0.b = (1-t0) * color0.b + t0 * color1.b;
        colort0.a = (1-t0) * color0.a + t0 * color1.a;

        const double t1 = (double)(i-start) / (end-start-1);
        std_msgs::ColorRGBA colort1;
        colort1.r = (1-t1) * color0.r + t1 * color1.r;
        colort1.g = (1-t1) * color0.g + t1 * color1.g;
        colort1.b = (1-t1) * color0.b + t1 * color1.b;
        colort1.a = (1-t1) * color0.a + t1 * color1.a;

        for (int j=0; j<joint_names_.size(); j++)
        {
            geometry_msgs::Point point;
            point.x = feature_(3*j+0, i);
            point.y = feature_(3*j+1, i);
            point.z = feature_(3*j+2, i);
            marker.points.push_back(point);
            marker.colors.push_back(colort0);

            point.x = feature_(3*j+0, i+1);
            point.y = feature_(3*j+1, i+1);
            point.z = feature_(3*j+2, i+1);
            marker.points.push_back(point);
            marker.colors.push_back(colort1);
        }
    }
    marker_array.markers.push_back(marker);

    publisher_.publish(marker_array);
}
