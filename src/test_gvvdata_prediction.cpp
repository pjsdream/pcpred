#include <ros/ros.h>

#include <pcpred/prediction/gvv_predictor.h>

#include <stdio.h>

#include <iostream>

using namespace pcpred;


int main(int argc, char** argv)
{
    int sequence_number = 1;
    if (argc >= 2)
        sequence_number = atoi(argv[1]);

    ros::init(argc, argv, "test_gvvdata_prediction");
    ROS_INFO("test_gvvdata_prediction");
    ros::NodeHandle n;



    const double timestep = 0.05;
    const double sensor_error = 0.01;
    const double collision_probability = 0.95;
    const int acceleration_inference_window_size = 5;
    const int prediction_frames = 3;

    ros::Rate rate(1.0 / timestep);



    GvvPredictor predictor(sequence_number);

    predictor.setTimestep(timestep);
    predictor.setSensorDiagonalCovariance(sensor_error * sensor_error);   // variance is proportional to square of sensing error
    predictor.setCollisionProbability(collision_probability);
    predictor.setAccelerationInferenceWindowSize(acceleration_inference_window_size);

    predictor.setMaximumIterations(5);
    predictor.setGradientDescentMaximumIterations(5);
    predictor.setGradientDescentAlpha(0.005);
    predictor.setHumanShapeLengthConstraintEpsilon(0.01);
    predictor.setCapsuleDivisor(4);

    predictor.setVisualizerTopic("gvv_prediction");

    // input sequence transformation
    // for example:
    //   predictor.rotate(M_PI / 2.0, Eigen::Vector3d(0.0, 0.0, 1.0));
    //   predictor.translate(Eigen::Vector3d(-1.0, 0.0, 0.0));

    while (true)
    {
        predictor.moveToNextFrame();
        predictor.predict(prediction_frames);

        printf("time = %8.4lf s\n", predictor.time());

        // to retrieve gaussian distribution
        for (int future_frame_index = 0; future_frame_index < prediction_frames; future_frame_index++)
        {
            std::vector<Eigen::Vector3d> mu;
            std::vector<Eigen::Matrix3d> sigma;

            predictor.getPredictedGaussianDistribution(future_frame_index, mu, sigma);

            for (int j=0; j<mu.size(); j++)
            {
                // probabilistic density function p(x[j]) ~ N(mu[j], sigma[j])
            }
        }

        // to visualize
        predictor.visualizePointcloud();
        predictor.visualizeHuman();
        predictor.visualizePredictionUpto(prediction_frames);

        fflush(stdout);
        rate.sleep();
    }

    return 0;
}

