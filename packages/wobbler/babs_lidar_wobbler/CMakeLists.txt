cmake_minimum_required(VERSION 2.8.3)
project(babs_lidar_wobbler)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next 2 lines to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})

#uncomment the following 4 lines to use the Eigen library
#find_package(cmake_modules REQUIRED)
#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
#add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# Libraries: uncomment the following and edit arguments to create a new library
# cs_add_library(my_lib src/my_lib.cpp)   

# Executables
cs_add_executable(motor_wobble src/motor_wobble.cpp)
cs_add_executable(wobbler_joint_static_tf src/wobbler_joint_static_tf.cpp)
cs_add_executable(wobbler_laser_tf src/wobbler_laser_tf.cpp)
cs_add_executable(hokuyo_pcl_stitcher src/hokuyo_pcl_stitcher.cpp)
cs_add_executable(wobbler_transformer src/wobbler_transformer.cpp)

cs_install()
cs_export()
    
