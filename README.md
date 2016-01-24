# pcpred

## Dynamic obstacle prediction with BVH motion file as input

* example code:
```c++
#include <pcpred/prediction/bvh_predictor.h>
using namespace pcpred;
...
BvhPredictor predictor("../data/bvh/walking.bvh");
predictor.setTimestep(0.1);
...
while (true)
{
  predictor.moveToNextFrame();

  predictor.visualizeHuman();

  double time = 0.0;
  while (time <= 0.3)
  {
    predictor.visualizePrediction(time);
    time += 0.05;
  }
}
```

More details in **pcpred/src/test_bvh_prediction.cpp**

## Dynamic obstacle prediction with point cluod input

* example code 1:
```c++
#include <pcpred/prediction/gvv_predictor.h>
using namespace pcpred;
...
GvvPredictor predictor(1); // sequence number
predictor.setTimestep(0.1);
...
while (true)
{
  predictor.moveToNextFrame();

  predictor.visualizePointcloud();
  predictor.visualizeHuman();

  double time = 0.0;
  while (time <= 0.3)
  {
    predictor.visualizePrediction(time);
    time += 0.05;
  }
}
```

* example code 2:
```c++
#include <pcpred/prediction/kinect_predictor.h>
using namespace pcpred;
...
KinectPredictor predictor(1); // sequence number
predictor.setTimestep(0.1);
...
while (true)
{
  predictor.moveToNextFrame();

  predictor.visualizePointcloud();
  predictor.visualizeHuman();

  double time = 0.0;
  while (time <= 0.3)
  {
    predictor.visualizePrediction(time);
    time += 0.05;
  }
}
```

More details in **pcpred/src/test_gvvdata_prediction.cpp** and **pcpred/src/test_kinect_prediction.cpp**

## How to run

1. Install ROS

2. Downlaod **Eigen3**
   and put the source folder into  
     **pcpred/include/**

   To run with point cloud data captured by MPI GVV group,
   download sequences from http://gvvperfcapeva.mpi-inf.mpg.de/public/InertialDepthTracker/index.php
   and unzip at  
     **pcpred/data/**

   Unzip C1C2.tar.gz and move C1/ and C2/ directories to
     **pcpred/data/**

3. Build with  
     **pcpred/CMakeLists.txt**

4. Launch **pcpred/launch/test.launch**  
   e.g.  **$ roslaunch pcpred test.launch**

5. Run test programs

  1. **BVH motion input**

     run **pcpred/bin/test_bvh_prediction**
     with the first argument the bvh filename  
     e.g.  **$ ./test_bvh_prediction ../data/bvh/walking.bvh**

  2. **MPI GVV point cloud input**

     run **pcpred/bin/test_gvvdata_prediction**
     with the first argument the sequence number  
     e.g.  **$ ./test_gvvdata_prediction 1**

  3. **Kinect point cloud input**

     run **pcpred/bin/test_kinect_prediction**
     with the first argument the sequence number
     e.g.  **$ ./test_kinect_prediction 1**
