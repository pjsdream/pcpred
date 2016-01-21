cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# pcl library setup
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenMP setup
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# header files
file(GLOB_RECURSE HEADER_FILES RELATIVE ${PROJECT_SOURCE_DIR} *.h)

# pcpred library
rosbuild_add_library(pcpred
src/detection/depth_frame_human_detector.cpp
src/detection/pointcloud_human_detector.cpp
src/prediction/point_predictor.cpp
src/prediction/points_predictor.cpp
src/prediction/pointcloud_human_predictor.cpp
src/prediction/bvh_predictor.cpp
src/prediction/gvv_predictor.cpp
src/shape/human.cpp
src/kalman_filter/point_obstacle_kf.cpp
src/pointcloud/pointcloud.cpp
src/visualization/visualizer.cpp
src/visualization/marker_visualizer.cpp
src/visualization/marker_array_visualizer.cpp
src/visualization/pointcloud_visualizer.cpp
src/import/bvh_importer.cpp
src/import/gvv_data_importer.cpp
src/import/openni_subscriber.cpp
src/util/chi_square.cpp
${HEADER_FILES}
)

# test application for point prediction
rosbuild_add_executable(test_point_prediction
src/test_point_prediction.cpp
)
target_link_libraries(test_point_prediction pcpred)

# test application for points prediction using bvh data
rosbuild_add_executable(test_bvh_prediction
src/test_bvh_prediction.cpp
)
target_link_libraries(test_bvh_prediction pcpred)

# test application for pointcloud human prediction
rosbuild_add_executable(test_pointcloud_prediction
src/test_pointcloud_prediction.cpp
)
target_link_libraries(test_pointcloud_prediction pcpred)

# test application for gvv pointcloud human prediction
rosbuild_add_executable(test_gvvdata_prediction
src/test_gvvdata_prediction.cpp
)
target_link_libraries(test_gvvdata_prediction pcpred)

# test application for gvv point cloud data import
rosbuild_add_executable(test_gvvdata_import
src/test_gvvdata_import.cpp
)
target_link_libraries(test_gvvdata_import pcpred)
target_link_libraries(test_gvvdata_import ${PCL_LIBRARIES})

# test application for bvh data import
rosbuild_add_executable(test_bvh_import
src/test_bvh_import.cpp
)
target_link_libraries(test_bvh_import pcpred)

# test application for openni raw image subscriber
rosbuild_add_executable(test_openni_subscriber
src/test_openni_subscriber.cpp
)
target_link_libraries(test_openni_subscriber pcpred)
