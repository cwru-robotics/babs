cmake_minimum_required(VERSION 2.8.3)
project(babs_motor_wobbler)

find_package(catkin_simple REQUIRED)

catkin_simple()

# Executables
cs_add_executable(dynamixel_motor_driver src/dynamixel_motor_driver.cpp src/ReadWrite.c src/dynamixel.c src/dxl_hal.c)

cs_install()
cs_export()
   
