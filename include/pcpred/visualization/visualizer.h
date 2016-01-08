#ifndef VISUALIZER_H
#define VISUALIZER_H


#include <ros/ros.h>


namespace pcpred
{

class Visualizer
{
public:

    explicit Visualizer(const char* topic);

protected:

    ros::Publisher publisher_;
};

}


#endif // VISUALIZER_H
