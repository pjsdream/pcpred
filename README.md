# pcpred

Dynamic obstacle prediction with BVH motion file as input

example code:
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

More details in
  pcpred/src/test_bvh_prediction.cpp


# How to run

1. Install ROS

2. Downlaod Eigen3
   and put the source folder into
     pcpred/include/

3. Build with pcpred/CMakeLists.txt

4. Launch pcpred/launch/test.launch
   e.g.  $ roslaunch pcpred test.launch

5. Run pcpred/bin/test_bvh_prediction
   with the first argument the bvh filename
   e.g.  $ ./test_bvh_prediction ../data/bvh/walking.bvh

