// Node-level integration testing for the BABS Wobbler
// Created Mar 22 2017 by Trent Ziemer
// Last updated Mar 22 by Trent Ziemer

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
/*
#include <pcl_ros/point_cloud.h> 
#include <pcl/point_types.h> 
*/

ros::NodeHandle * nh_ptr;

class SubscriptionVerifier
{
public:
    SubscriptionVerifier();
    void verifyScan(const sensor_msgs::LaserScan::ConstPtr&);
    bool checkSubscription();
private:
    int time_to_wait;
    bool scan_verified;
};

SubscriptionVerifier::SubscriptionVerifier()
{
    time_to_wait = 50;
    scan_verified = false;
}

void SubscriptionVerifier::verifyScan(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    if(1)
    {
        scan_verified = true;
    }
}

bool SubscriptionVerifier::checkSubscription()
{
    int count = 0;
    ros::Rate count_rate(10);
    while(count < time_to_wait)
    {
        if(scan_verified)
        {
            return true;
        }
        ros::spinOnce();
        count_rate.sleep();
        count++;
    }
    return false;
}

int main(int argc, char** argv) 
{
    // CHECK THREE THINGS
    //    DATA STACK
    //    TRANSFORM STACK
    //    MOTOR STACK
    //    (and then all three, or maybe superfluous)

    // Standard ROS stuff
    ros::init(argc, argv, "wobbler_integration_test");
    ros::NodeHandle nh;
    nh_ptr = &nh;

    // Keep track of how many tests we run, and how many are successful
    int tests_passed = 0;
    int total_tests = 0;

    ROS_INFO("Starting testing data and point cloud stack");
    ROS_INFO("Testing hokuyo_node");

    total_tests++;
    std::string hokuyo_scan_name;

    double dummy;

    while(ros::ok())
    {
        if(nh_ptr->getParam("/wobbler_integration_test/start_testing", dummy))
        {
            break;
        }
    }
    if(!nh_ptr->getParam("/wobbler_integration_test/hokuyo_scan_name", hokuyo_scan_name))
    {
        ROS_WARN("Failed to get wobbler hokuyo scan topic name!");
    }
    else
    {
        total_tests++;
        SubscriptionVerifier hokuyoVerifier;

        ros::Subscriber hokuyo_sub = nh_ptr->subscribe(hokuyo_scan_name, 1, &SubscriptionVerifier::verifyScan, &hokuyoVerifier);

        if(!hokuyoVerifier.checkSubscription())
        {
            ROS_WARN("Failed to verify correct hokuyo data!");
        }
    }

	if(total_tests == tests_passed)
	{
		ROS_INFO("INTEGRATION TESTING PASSED");
	}
	else
	{
		ROS_WARN("INTEGRATION TESTING FAILED: %d tests failed out of %d total!", (total_tests - tests_passed), total_tests);
	}


    ROS_INFO("Closing out of tests...");

    return 0;
}
