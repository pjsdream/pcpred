#include <ros/ros.h>

#include <pcpred/prediction/kinect_predictor.h>
#include <pcpred/visualization/pointcloud_visualizer.h>

#include <Eigen/SVD>

#include <stdio.h>

using namespace pcpred;


int main(int argc, char** argv)
{
    int sequence_number = 1;
    if (argc >= 2)
        sequence_number = atoi(argv[1]);

    ros::init(argc, argv, "figures");
    ROS_INFO("figures");
    ros::NodeHandle n;


    const double timestep = 0.05;
    const double prediction_timestep = 0.05;
    const double sensor_error = 0.005;
    const double collision_probability = 0.95;
    const int acceleration_inference_window_size = 5;
    const int prediction_frames = 6;

    ros::Rate rate(1.0 / timestep);



    KinectPredictor predictor(sequence_number);

    predictor.setTimestep(timestep);
    predictor.setSensorDiagonalCovariance(sensor_error * sensor_error);   // variance is proportional to square of sensing error
    predictor.setCollisionProbability(collision_probability);
    predictor.setAccelerationInferenceWindowSize(acceleration_inference_window_size);

    predictor.setMaximumIterations(5);
    predictor.setGradientDescentMaximumIterations(5);
    predictor.setGradientDescentAlpha(0.005);
    predictor.setHumanShapeLengthConstraintEpsilon(0.01);

    predictor.translate(Eigen::Vector3d(0, 0, -0.1));
    predictor.setVisualizerTopic("figures");

    int frame = 0;
    while (true)
    {
        predictor.moveToNextFrame();

        frame++;
        if (frame == 10)
        {
            FILE* fp = fopen("../data/figures/circles.txt", "w");

            // to visualize
            predictor.visualizePointcloud();
            predictor.visualizeHuman();
            for (int future_frame_index = 0; future_frame_index < prediction_frames; future_frame_index++)
                predictor.visualizePrediction(future_frame_index * prediction_timestep);

            // to retrieve gaussian distribution
            std::vector<Eigen::Vector3d> mu;
            std::vector<Eigen::Matrix3d> sigma;
            std::vector<double> radius;

            predictor.getPredictedGaussianDistribution(0, mu, sigma, radius);

            for (int j=0; j<mu.size(); j++)
            {
                // to YZ plane
                const double y = mu[j](1);
                const double z = mu[j](2);
                const double r = radius[j];

                const Eigen::Matrix2d sigma2 = sigma[j].block(1, 1, 2, 2);
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(sigma2, Eigen::ComputeThinU | Eigen::ComputeThinV);
                Eigen::MatrixXd U = svd.matrixU();
                if (U.determinant() < 0.)
                    U.col(1) *= -1.;

                /*
                fprintf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
                        y, z, r,
                        U(0,0), U(1,0), svd.singularValues()(0),
                        U(0,1), U(1,1), svd.singularValues()(1));
                        */

                fprintf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
                        y, z, r,
                        sigma2(0,0), sigma2(1,0), sigma2(0,1), sigma2(1,1));
            }

            fclose(fp);

            const Pointcloud& pointcloud = predictor.pointcloud();
            fp = fopen("../data/figures/pointcloud.txt", "w");
            for (int i=0; i<pointcloud.size(); i++)
            {
                const Eigen::Vector3d& v = pointcloud.point(i);
                fprintf(fp, "%lf,%lf\n", v(1), v(2));
            }
            fclose(fp);

            break;
        }
    }

    return 0;
}

