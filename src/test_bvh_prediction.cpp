#include <ros/ros.h>

#include <stdio.h>
#include <pcpred/pcpred.h>

#include <Eigen/Dense>

#include <string>

using namespace pcpred;


int main(int argc, char** argv)
{
    std::string filename = "../data/bvh/walking.bvh";
    if (argc >= 2)
        filename = argv[1];

    ros::init(argc, argv, "test_bvh_prediction");

    ROS_INFO("test_bvh_prediction");

    BvhImporter importer;
    MarkerVisualizer visualizer("bvh_prediction_test");

    if (!importer.import(filename.c_str()))
    {
        ROS_ERROR("bvh file does not exist [%s]", filename.c_str());
        return 0;
    }

    // centimeters to meters
    importer.scale(0.01);

    // rotate from Y-up to Z-up
    importer.rotate(M_PI / 2.0, Eigen::Vector3d(1.0, 0.0, 0.0));

    const int num_frames = importer.numFrames();
    const int num_joints = importer.numJoints();
    ros::Rate rate(importer.rate());

    for (int joint_index = 0; joint_index < num_joints; joint_index++)
        visualizer.clearUptoCapacity(importer.jointName(joint_index).c_str());

    const double fps = importer.rate();
    const double timestep = 1. / fps;
    const double sensor_error = 0.001;

    const int acceleration_inference_window_size = 15;
    const double radius = 0.1;
    const int prediction_frames = 5;

    while (true)
    {
        PointsPredictor predictor(num_joints);
        predictor.setTimestep(timestep);
        predictor.setSensorDiagonalCovariance(sensor_error * sensor_error); // variance is proportional to square of sensing error
        predictor.setAccelerationInferenceWindowSize(acceleration_inference_window_size);

        for (int frame_index = 0; frame_index < num_frames; frame_index++)
        {
            std::vector<Eigen::Vector3d> points;
            for (int joint_index = 0; joint_index < num_joints; joint_index++)
            {
                const Eigen::Affine3d transformation = importer.jointTransformation(frame_index, joint_index);
                visualizer.drawSphere("robot", joint_index, transformation.translation(), radius);
                points.push_back(transformation.translation());
            }

            predictor.observe(points);
            predictor.predict(prediction_frames);

            for (int prediction_index = 0; prediction_index < prediction_frames; prediction_index++)
            {
                for (int joint_index = 0; joint_index < num_joints; joint_index++)
                {
                    Eigen::Vector3d mu;
                    Eigen::Matrix3d sigma;

                    predictor.getPredictionResult(prediction_index, joint_index, mu, sigma);
                    visualizer.drawGaussianDistribution("prediction", prediction_index * num_joints + joint_index, mu, sigma, 0.95, radius);
                }
            }

            rate.sleep();
        }
    }

    return 0;
}

