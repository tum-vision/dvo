cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# Eigen3 library
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

# dynamic reconfigure
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

rosbuild_add_library(${PROJECT_NAME} 
    src/camera_base.cpp
    src/camera_dense_tracking.cpp
    
    src/visualization/ros_camera_trajectory_visualizer.cpp
)

rosbuild_add_library(camera_tracker_nodelet
    src/camera_tracker_nodelet.cpp
)

target_link_libraries(camera_tracker_nodelet
    ${PROJECT_NAME}
)

rosbuild_add_executable(camera_tracker
     src/camera_tracker_node.cpp
)

target_link_libraries(camera_tracker
    ${PROJECT_NAME}
)