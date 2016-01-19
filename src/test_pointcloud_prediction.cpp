#include <ros/ros.h>

#include <Eigen/Dense>
#include <vector>

#include <pcpred/import/gvv_data_importer.h>
#include <pcpred/visualization/pointcloud_visualizer.h>
#include <pcpred/visualization/marker_array_visualizer.h>
#include <pcpred/prediction/pointcloud_human_predictor.h>

#include <iostream>

using namespace pcpred;


int main(int argc, char** argv)
{
    int sequence_number = 1;
    if (argc >= 2)
        sequence_number = atoi(argv[1]);

    ros::init(argc, argv, "test_pointcloud_prediction");

    ROS_INFO("test_pointcloud_prediction");

    PointcloudVisualizer pointcloud_visualizer("pointcloud_test");

    MarkerArrayVisualizer visualizer("camera");

    PointcloudHumanPredictor predictor;
    predictor.loadHumanShapeFromFile("../data/human/S1.txt");
    predictor.setMaximumIterations(5);
    predictor.setGradientDescentMaximumIterations(5);
    predictor.setGradientDescentAlpha(0.0005);
    predictor.setHumanShapeLengthConstraintEpsilon(0.01);
    predictor.setCapsuleDivisor(4);
    predictor.setVisualizerTopic("pointcloud_human_test");

    GVVDataImporter importer;

    ros::Rate rate(33);

    int frame = 0;
    while (importer.import(sequence_number, frame, false))
    {
        Pointcloud pointcloud( importer.pointcloud() );
        pointcloud.rotate(M_PI / 2.0, Eigen::Vector3d(1, 0, 0));

        const Eigen::AngleAxisd rotation( M_PI / 2.0, Eigen::Vector3d(1, 0, 0) );

        Eigen::Vector3d camera_position = importer.cameraPosition();
        camera_position = rotation * camera_position;

        std::vector<Eigen::Vector3d> camera_endpoints = importer.cameraEndpoints();
        for (int i=0; i<camera_endpoints.size(); i++)
            camera_endpoints[i] = rotation * camera_endpoints[i];

        predictor.observe(camera_position, pointcloud);

        pointcloud_visualizer.drawPointcloud(pointcloud);
        predictor.visualizeHuman();

        // visualize camera
        std::vector<Eigen::Vector3d> points;
        points.push_back(camera_endpoints[0]);
        points.push_back(camera_endpoints[1]);
        points.push_back(camera_endpoints[3]);
        points.push_back(camera_endpoints[2]);
        points.push_back(camera_endpoints[0]);
        visualizer.drawLineStrip("far", points);

        points.clear();
        points.push_back(camera_endpoints[4]);
        points.push_back(camera_endpoints[5]);
        points.push_back(camera_endpoints[7]);
        points.push_back(camera_endpoints[6]);
        points.push_back(camera_endpoints[4]);
        visualizer.drawLineStrip("near", points);

        points.clear();
        for (int i=0; i<4; i++)
        {
            points.push_back(camera_endpoints[i]);
            points.push_back(camera_endpoints[i + 4]);
        }
        visualizer.drawLineList("side", points);

        visualizer.drawSphere("position", camera_position, 0.1);

        printf("frame %4d  #points = %5d\n", frame, pointcloud.size());
        fflush(stdout);

        rate.sleep();

        frame++;
    }

    return 0;
}
