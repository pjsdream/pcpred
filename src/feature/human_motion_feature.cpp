#include <pcpred/feature/human_motion_feature.h>

#include <stdio.h>

using namespace pcpred;


HumanMotionFeature::HumanMotionFeature()
{
    setVisualizerTopic("human_motion");

    setCurveShape(5, 1.0);
}

void HumanMotionFeature::setVisualizerTopic(const std::string &topic)
{
    ros::NodeHandle nh;
    publisher_.shutdown();
    publisher_ = nh.advertise<visualization_msgs::MarkerArray>(topic, 100);
}

void HumanMotionFeature::setJointNames(const std::vector<std::string>& joints)
{
    map_joint_to_index_.clear();
    streams_.resize(joints.size());
    curves_.resize(joints.size());

    for (int i=0; i<joints.size(); i++)
    {
        map_joint_to_index_[joints[i]] = i;
        streams_[i].clear();
        streams_[i].setCurveShape(num_pieces_, duration_);
    }
}

void HumanMotionFeature::addLink(const std::string& joint1, const std::string& joint2)
{
    links_.push_back( std::make_pair( map_joint_to_index_[joint1], map_joint_to_index_[joint2] ) );
}

void HumanMotionFeature::setCurveShape(int num_pieces, double duration)
{
    num_pieces_ = num_pieces;
    duration_ = duration;

    for (int i=0; i<streams_.size(); i++)
        streams_[i].setCurveShape(num_pieces, duration);
}

void HumanMotionFeature::observe(const std::string& joint_name, double time, const Eigen::Vector3d& joint_position)
{
    if (map_joint_to_index_.find(joint_name) != map_joint_to_index_.end())
    {
        const int index = map_joint_to_index_[joint_name];

        streams_[index].push(time, joint_position);
        streams_[index].fit();
        curves_[index] = streams_[index].toHermiteCurve();
    }
}

Eigen::VectorXd HumanMotionFeature::toFeature(FeatureType feature_type)
{
    switch (feature_type)
    {
    case FEATURE_TYPE_ABSOLUTE_POSITION:
        return toFeatureAbsolutePosition();

    default:
        fprintf(stderr, "Error: unknown feature type\n");
        fflush(stderr);
        return Eigen::Vector3d::Zero();
    }
}

Eigen::VectorXd HumanMotionFeature::toFeatureAbsolutePosition()
{
    const int num_curves = curves_.size();
    const int feature_size = curves_[0].featureSize();

    Eigen::VectorXd x( num_curves * feature_size );

    for (int i=0; i<curves_.size(); i++)
        x.block(i * feature_size, 0, feature_size, 1) = curves_[i].toFeature();

    return x;
}

void HumanMotionFeature::visualizeHumanMotion()
{
    visualization_msgs::MarkerArray marker_array;

    std_msgs::ColorRGBA white, black, red, green;
    white.a = black.a = red.a = green.a = 1.;

    white.r = white.g = white.b = 1.;
    black.r = black.g = black.b = 0.;
    red.r = 1.;
    red.g = red.b = 0.;
    green.g = 1.;
    green.r = green.b = 0.;

    appendTraceMarkers(0.025, white, black, marker_array);
    appendJointMarkers(0, 0.05, white, marker_array);
    appendJointMarkers(num_pieces_, 0.05, black, marker_array);
    appendSkeletonMarkers(0, 0.025, red, marker_array);
    appendSkeletonMarkers(num_pieces_, 0.025, green, marker_array);

    publisher_.publish(marker_array);
}

void HumanMotionFeature::appendTraceMarkers(double size, const std_msgs::ColorRGBA& color0, const std_msgs::ColorRGBA& color1, visualization_msgs::MarkerArray& marker_array)
{
    const int res = 16;
    const int num_points = num_pieces_ * res + 1;

    visualization_msgs::Marker marker;
    std_msgs::ColorRGBA color;

    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time();
    marker.ns = "trace";
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;
    marker.pose.orientation.w = 1.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;
    marker.scale.x = size;

    for (int i=0; i < num_points; i++)
    {
        const double t = (double)i / (num_points - 1);
        color.r = (1-t) * color0.r + t * color1.r;
        color.g = (1-t) * color0.g + t * color1.g;
        color.b = (1-t) * color0.b + t * color1.b;
        color.a = (1-t) * color0.a + t * color1.a;
        marker.colors.push_back(color);
    }

    for (int i=0; i<curves_.size(); i++)
    {
        marker.id = i;

        marker.points.clear();

        for (int j=0; j<num_points; j++)
        {
            const double t = (double)j / (num_points - 1) * num_pieces_;
            const Eigen::Vector3d x = curves_[i](t);

            geometry_msgs::Point point;
            point.x = x(0);
            point.y = x(1);
            point.z = x(2);
            marker.points.push_back(point);
        }

        marker_array.markers.push_back(marker);
    }
}

void HumanMotionFeature::appendJointMarkers(double time, double size, const std_msgs::ColorRGBA& color, visualization_msgs::MarkerArray& marker_array)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time();
    marker.ns = "joints_" + std::to_string(time);
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;
    marker.pose.orientation.w = 1.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    marker.color = color;

    for (int i=0; i<curves_.size(); i++)
    {
        marker.id = i;

        const Eigen::Vector3d x = curves_[i](time);

        geometry_msgs::Point point;
        point.x = x(0);
        point.y = x(1);
        point.z = x(2);
        marker.points.push_back(point);
    }

    marker_array.markers.push_back(marker);
}

void HumanMotionFeature::appendSkeletonMarkers(double time, double size, const std_msgs::ColorRGBA& color, visualization_msgs::MarkerArray& marker_array)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time();
    marker.ns = "skeleton_" + std::to_string(time);
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;
    marker.pose.orientation.w = 1.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    marker.color = color;

    for (int i=0; i<links_.size(); i++)
    {
        marker.id = i;

        const int i0 = links_[i].first;
        const int i1 = links_[i].second;

        const Eigen::Vector3d x0 = curves_[i0](time);
        const Eigen::Vector3d x1 = curves_[i1](time);

        geometry_msgs::Point point;
        point.x = x0(0);
        point.y = x0(1);
        point.z = x0(2);
        marker.points.push_back(point);
        point.x = x1(0);
        point.y = x1(1);
        point.z = x1(2);
        marker.points.push_back(point);
    }

    marker_array.markers.push_back(marker);
}
