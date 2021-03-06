#include <ros/ros.h>

#include <Eigen/Dense>
#include <vector>

#include <pcpred/pcpred.h>

#include <iostream>

using namespace pcpred;


typedef std::vector<Eigen::Vector3d> Path;


Path produceObstaclePathStatic(int frame_count)
{
    Path path;

    for (int i=0; i<frame_count; i++)
        path.push_back(Eigen::Vector3d(0, 0, 0));

    return path;
}

Path produceObstaclePathSine(int frame_count, double timestep, double speed)
{
    Path path;

    for (int i=0; i<frame_count; i++)
    {
        double t = timestep * speed * i;
        path.push_back(Eigen::Vector3d(t, sin(t), 0));
    }

    return path;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_point_prediction");

    ROS_INFO("test_point_prediction");

    const double fps = 10;
    const double timestep = 1. / fps;
    const double speed = 1.0;
    const double sensor_error = 0.001;
    const double duration = 10.;

    const int frame_count = duration * fps;
    const int predict_frame_count = 20;
    const int acceleration_inference_window_size = 5;
    const double radius = 0.1;

    PointPredictor predictor;
    predictor.setObservationTimestep(timestep);
    predictor.setSensorDiagonalCovariance(sensor_error * sensor_error); // variance is proportional to square of sensing error
    predictor.setAccelerationInferenceWindowSize(acceleration_inference_window_size);

    MarkerVisualizer visualizer("prediction_test");
    visualizer.clearUptoCapacity("path");
    visualizer.clearUptoCapacity("prediction");

    ros::Rate rate(fps);

    // static path
    //Path path = produceObstaclePathStatic(frame_count);

    // sine path
    Path path = produceObstaclePathSine(frame_count, timestep, speed);

    ros::Duration(3.0).sleep();
    for (int j=0; j<path.size(); j++)
        visualizer.drawSphere("path", j, path[j], radius);

    for (int i=0; i<path.size(); i++)
    {
        predictor.observe(path[i]);

        for (int j=0; j<predict_frame_count; j++)
        {
            Eigen::Vector3d mu;
            Eigen::Matrix3d sigma;

            predictor.predict(j * timestep, mu, sigma);
            visualizer.drawGaussianDistribution("prediction", j, mu, sigma, 0.95, radius);

            std::cout << "Prediction at time " << i << " of time " << i+j << std::endl;
            std::cout << "observation = " << path[i].transpose() << std::endl;
            std::cout << "mu = " << mu.transpose() << std::endl;
            std::cout << "sigma = " << std::endl << sigma << std::endl;
            std::cout << std::endl;
        }
        std::cout.flush();
        rate.sleep();
    }

    return 0;
}

