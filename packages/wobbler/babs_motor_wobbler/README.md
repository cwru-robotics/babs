# babs_wobbler_motor
Heavily based on Dr. Wyatt Newmans (CWRU) ROS package baxter_gripper
This package combines some Dynamixel communications C-code with a ROS node.  A Dynamixel MX-64 is assumed, with an input command range of 0-4095 for one revolution.

Note: Make sure the chosen USB port is read/writable (e.g. /dev/ttyUSB0).

## Example usage
`rosrun babs_wobbler_motor dynamixel_motor_driver`
Note: You can use command line parameters to set motor ID, baud rate, and tty device for your motor.

This will attempt to communicate to a Dynamixel motor with default values of:
* motor_id = 1
* baudnum code = 1 (1,000,000 BPS)
* ttynum=0 (/dev/ttyUSB0)

The node will subscribe to topic "dynamixel_motor1_cmd" for input commands in range 0-4095, and it will publish to topic "dynamixel_motor1_ang" with the current reported motor angle.  However, there are frequent communications faults.  If a read fault is detected, the published angle will be > 4096.  This should be inspected, to determine if the angle may be trusted.

Can send manual commands with 
`rostopic pub dynamixel_motor1_cmd std_msgs/Int16 3600`

Alternatively, you can set 2 motor parameters on the ROS param server, and set the variable within this source code to use two different (front and rear) motors

Input (motor command) topics would then be:
"front_wobbler/cmd" and "rear_wobbler/cmd"

Output (motor angle) topics that this node subscribes to would then be:
"front_wobbler/angle" and "rear_wobbler/angle"


Final note: The main node also uses the three C libraries:
"src/ReadWrite.c", "src/dynamixel.c", and "src/dxl_hal.c"



    
