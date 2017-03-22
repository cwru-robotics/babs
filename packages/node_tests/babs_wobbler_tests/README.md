# babs_wobbler_tests

A collection of ROS node unit- and integration-level testing material for the BABS wobbler package

## Example usage
Each launch file in the launch directory can be called via "roslaunch babs_wobbler_tests xxx_xxx.launch"

Not recommended to run nodes themselves directly; most of these test nodes are setup to work within the relevant launch file.
    
## Tests Available

integration.launch - Checks inter-node communications and consistency and functionality for the main purpose of the nodes as start by the "start_wobbler.launch" point

lidar_calibration.launch - Infrastructure there, but not implemented any functionality (should be updated with a plane matching algorithm). Intended to test the functional testing node.

motor_wobble.launch - Checks whether the motor_wobble node works with a rosbag (as of right now, should update soon)

hokuyo_pcl_stitcher.launch - NIMPL

wobbler_joint_static_tf.launch - NIMPL

wobbler_laser_tf.launch - NIMPL

wobbler_transformer.launch - NIMPL
