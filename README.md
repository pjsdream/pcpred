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
  predictor.predict(5);

  predictor.visualizeHuman();
  predictor.visualizePredictionUpto(5);
}
```

More details in **pcpred/src/test_bvh_prediction.cpp**

## Dynamic obstacle prediction with point cluod input

* example code:
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
  predictor.predict(5);

  predictor.visualizePointcloud();
  predictor.visualizeHuman();
  predictor.visualizePredictionUpto(5);
}
```

More details in **pcpred/src/test_gvv_prediction.cpp**

## How to run

1. Install ROS

2. Downlaod **Eigen3**
   and put the source folder into  
     **pcpred/include/**

   To run with point cloud data captured by MPI GVV group,
   download sequences from http://gvvperfcapeva.mpi-inf.mpg.de/public/InertialDepthTracker/index.php
   and unzip at  
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

