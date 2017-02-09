#crio_code
This directory contains all code that will run the cRIO Power PC. This will handle the PSO, GPS and
Encoders.  However USB interface instead of the cRIO.

#gps-msgs
gps-msgs contains all the different msgs that can be recieved from the GPS module.

#novatel
novatel contains the ROS node for the GPS module on BABS
'''
'''

#odom_filters
odom filters contains the launch commands for the device path and odom filters.
'''
roslaunch odom_filters filters.launch
'''

#razor_imu_9of
roboteq contains the ROS node for the motor controller on BABS.
'''
roslaunch roboteq_driver motor_controller.launch
'''

#sentry
'''
'''

#usb_cams
usb_cams contains the ROS node for the 180 degree usb cameras.
'''
'''

#wobbler
Contains wobbler drivers, processors, testers, and point cloud processing for Hokyu LIDAR + Dynamixel motor based 3D point cloud wobblers for 3d-informed navigation
