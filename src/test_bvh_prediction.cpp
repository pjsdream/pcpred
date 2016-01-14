#include <ros/ros.h>
#include <stdio.h>
#include <string>


#include <pcpred/prediction/bvh_predictor.h>

using namespace pcpred;


int main(int argc, char** argv)
{
    std::string filename = "../data/bvh/walking.bvh";
    if (argc >= 2)
        filename = argv[1];

    ros::init(argc, argv, "test_bvh_prediction");
    ROS_INFO("test_bvh_prediction");

    ros::NodeHandle n;



    /*
    const double timestep = 0.01;               // 0.01 s
    const double sensor_error = 0.001;          // 1 mm
    const double collision_probability = 0.95;  // 95%
    const int acceleration_inference_window_size = 25;
    const int prediction_frames = 15;
    */

    const double timestep = 0.05;               // 0.05 s
    const double sensor_error = 0.001;          // 1 mm
    const double collision_probability = 0.95;  // 95%
    const int acceleration_inference_window_size = 5;
    const int prediction_frames = 3;



    ros::Rate rate(1. / timestep);



    // initialize predictor
    BvhPredictor predictor(filename.c_str());  // load bvh file
    predictor.setTimestep(timestep);
    predictor.setSensorDiagonalCovariance(sensor_error * sensor_error);   // variance is proportional to square of sensing error
    predictor.setCollisionProbability(collision_probability);
    predictor.setAccelerationInferenceWindowSize(acceleration_inference_window_size);
    predictor.setVisualizerTopic("bvh_prediction_test");

    // input sequence transformation
    // for example:
    //   predictor.rotate(M_PI / 2.0, Eigen::Vector3d(0.0, 0.0, 1.0));
    //   predictor.translate(Eigen::Vector3d(-1.0, 0.0, 0.0));


    while(true)
    {
        predictor.moveToNextFrame();
        predictor.predict(prediction_frames);

        printf("time = %8.4lf s\n", predictor.time());

        for (int future_frame_index = 0; future_frame_index < prediction_frames; future_frame_index++)
        {
            std::vector<Eigen::Vector3d> centers;
            std::vector<Eigen::Matrix3d> A;

            predictor.getPredictedEllipsoids(future_frame_index, centers, A);

            for (int j=0; j<centers.size(); j++)
            {
                // an ellipsoid is defined by
                //   the center:                centers[j]
                //   the principal axes:        the eigenvectors of A[j]
                //   the length principal axes: the eigenvalues of A[j]
            }
        }


        predictor.visualizeHuman();
        predictor.visualizePredictionUpto(prediction_frames);


        fflush(stdout);
        rate.sleep();
    }

    return 0;
}

