#include <ros/ros.h>

#include <Eigen/Dense>
#include <vector>

#include <pcpred/import/gvv_data_importer.h>
#include <pcpred/visualization/pointcloud_visualizer.h>
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

    PointcloudHumanPredictor predictor;
    predictor.loadHumanShapeFromFile("../data/human/S1.txt");
    predictor.setMaximumIterations(10);
    predictor.setGradientDescentMaximumIterations(10);
    predictor.setGradientDescentAlpha(0.0001);
    predictor.setHumanShapeLengthConstraintEpsilon(0.01);
    predictor.setCapsuleDivisor(4);
    predictor.setVisualizerTopic("pointcloud_human_test");

    GVVDataImporter importer;

    ros::Rate rate(33);

    int frame = 0;
    while (importer.import(sequence_number, frame, false))
    {
        printf("frame %4d", frame);

        Pointcloud pointcloud( importer.pointcloud() );
        pointcloud.rotate(M_PI / 2.0, Eigen::Vector3d(1, 0, 0));

        predictor.observe(pointcloud);

        pointcloud_visualizer.drawPointcloud(pointcloud);
        predictor.visualizeHuman();

        printf("  #points = %5d", pointcloud.size());
        fflush(stdout);

        rate.sleep();

        frame++;
    }

    return 0;
}
