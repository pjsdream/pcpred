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

# header/src files
file(GLOB_RECURSE HEADER_FILES ${PROJECT_SOURCE_DIR}/include/pcpred/*.h)
file(GLOB_RECURSE SOURCE_FILES ${PROJECT_SOURCE_DIR}/src/*/*.cpp)

# pcpred library
rosbuild_add_library(pcpred
${SOURCE_FILES}
${HEADER_FILES}
)


# test application for openni raw image subscriber
rosbuild_add_executable(test_openni_subscriber
src/test_openni_subscriber.cpp
)
target_link_libraries(test_openni_subscriber pcpred)

# test application for openni record playback
rosbuild_add_executable(test_openni_playback
src/test_openni_playback.cpp
)
target_link_libraries(test_openni_playback pcpred)

# test application for kinect prediction
rosbuild_add_executable(test_kinect_prediction
src/test_kinect_prediction.cpp
)
target_link_libraries(test_kinect_prediction pcpred)



# test application for gpr tutorial
rosbuild_add_executable(test_gpr_tutorial
src/test_gpr_tutorial.cpp
)
target_link_libraries(test_gpr_tutorial pcpred)
