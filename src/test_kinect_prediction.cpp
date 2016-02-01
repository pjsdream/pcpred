#include <ros/ros.h>

#include <pcpred/prediction/kinect_predictor.h>
#include <pcpred/visualization/pointcloud_visualizer.h>

#include <stdio.h>

using namespace pcpred;


int main(int argc, char** argv)
{
    int sequence_number = 1;
    if (argc >= 2)
        sequence_number = atoi(argv[1]);

    ros::init(argc, argv, "test_kinect_prediction");
    ROS_INFO("test_kinect_prediction");
    ros::NodeHandle n;


    const double timestep = 0.05;
    const double prediction_timestep = 0.05;
    double sensor_error = 0.005;
    double collision_probability = 0.95;
    const int acceleration_inference_window_size = 5;
    const int prediction_frames = 20;

    ros::Rate rate(1.0 / timestep);
    //ros::Rate rate(1.0);


    n.param("/itomp_planner/sensor_error", sensor_error, 0.005);
    n.param("/itomp_planner/collision_probability", collision_probability, 0.95);




    KinectPredictor predictor(sequence_number);

    predictor.setTimestep(timestep);
    predictor.setSensorDiagonalCovariance(sensor_error * sensor_error);   // variance is proportional to square of sensing error
    predictor.setCollisionProbability(collision_probability);
    predictor.setAccelerationInferenceWindowSize(acceleration_inference_window_size);

    predictor.setMaximumIterations(5);
    predictor.setGradientDescentMaximumIterations(5);
    predictor.setGradientDescentAlpha(0.005);
    predictor.setHumanShapeLengthConstraintEpsilon(0.01);

    predictor.setVisualizerTopic("kinect_prediction");

    // input sequence transformation
    // for example:
    //   predictor.rotate(M_PI / 2.0, Eigen::Vector3d(0.0, 0.0, 1.0));
    //   predictor.translate(Eigen::Vector3d(-1.0, 0.0, 0.0));

    const Eigen::Vector3d pointcloud_translates[] =
    {
        Eigen::Vector3d(0.1, -0.7, -0.9),
        Eigen::Vector3d(0.18, 1, -0.9),
        Eigen::Vector3d(0, -0.5, -0.9),
        Eigen::Vector3d(-0.2, 0.7, -0.9),
        Eigen::Vector3d(0.75, -0.9, -0.9),
        Eigen::Vector3d(0.75, -0.8, -0.9),
        Eigen::Vector3d(5.0, -0.0, -0.9),
        Eigen::Vector3d(5.0, -0.0, -0.9),
        Eigen::Vector3d(0.8, 2.5, -0.6),
        Eigen::Vector3d(0.7, 1.7, -0.7),
    };
    const double z_rotations[] =
    {
        0.0,
        0.0,
        0.0,
        0.0,
        -30.0 / 180.0 * M_PI,
        -15.0 / 180.0 * M_PI,
        0.0,
        0.0,
        -90.0 / 180.0 * M_PI,
        -90.0 / 180.0 * M_PI,
    };

    // transform
    predictor.translate( Eigen::Vector3d(0.4, 0.0, 0.0) );
    predictor.rotate( z_rotations[sequence_number - 1], Eigen::Vector3d(0, 0, 1) );
    predictor.translate( Eigen::Vector3d(-0.4, 0.0, 0.0) );
    predictor.translate( pointcloud_translates[sequence_number - 1] );


    for (int i = 3; i > 0; i--)
    {
        printf("%d\n", i);
        fflush(stdout);
        sleep(1);
    }


    while (true)
    {
        predictor.moveToNextFrame();

        // to retrieve gaussian distribution
        for (int future_frame_index = 0; future_frame_index < prediction_frames; future_frame_index++)
        {
            std::vector<Eigen::Vector3d> mu;
            std::vector<Eigen::Matrix3d> sigma;
            std::vector<double> radius;

            predictor.getPredictedGaussianDistribution(future_frame_index * prediction_timestep, mu, sigma, radius);

            for (int j=0; j<mu.size(); j++)
            {
                // probabilistic density function of sphere's center p(x[j]) ~ N(mu[j], sigma[j])
                // radius of sphere is radius[j]
            }
        }

        // to visualize
        predictor.visualizePointcloud();
        predictor.visualizeHuman();

        for (int future_frame_index = 0; future_frame_index < prediction_frames; future_frame_index++)
            predictor.visualizePrediction(future_frame_index * prediction_timestep);

        fflush(stdout);
        rate.sleep();
    }

    return 0;
}

