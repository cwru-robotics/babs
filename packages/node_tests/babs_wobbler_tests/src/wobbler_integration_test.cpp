// Node-level integration testing for the BABS Wobbler
// Created Mar 22 2017 by Trent Ziemer
// Last updated Mar 24 by Trent Ziemer

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/point_cloud.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::NodeHandle * nh_ptr;

class SubscriptionVerifier
{
public:
    SubscriptionVerifier();
    void verifyScan(const sensor_msgs::LaserScan::ConstPtr&);
    void verifyCloud(const PointCloud::ConstPtr&);
    bool checkSubscription();
private:
    int time_to_wait;
    bool scan_verified;
    bool cloud_verified;
};

SubscriptionVerifier::SubscriptionVerifier()
{
    time_to_wait = 3000; // milliseconds
    scan_verified = false;
}

void SubscriptionVerifier::verifyScan(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    if(1)
    {
        ROS_INFO("SCAN VERIFIED SET TO TRUE");
        scan_verified = true;
    }
}

void SubscriptionVerifier::verifyCloud(const PointCloud::ConstPtr& point_cloud)
{
    if(1)
    {
        ROS_INFO("CLOUD VERIFIED SET TO TRUE");
        cloud_verified = true;
    }
}

bool SubscriptionVerifier::checkSubscription()
{
    int count = 0;
    ros::Rate count_rate(1000); // milliseconds
    while(count < time_to_wait)
    {
        ROS_INFO("CHECKING SCAN || CLOUD VERIFIED IF TRUE");
        if(scan_verified || cloud_verified)
        {
            ROS_INFO("SCAN || CLOUD VERIFIED WAS INDEED TRUE");
            return true;
        }
        ros::spinOnce();
        count_rate.sleep();
        count++;
    }
    return false;
}

/*

// Test Proc?
void runTest(std::string node_name)
{

}

*/

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

    double dummy;

    // Wait for us to get the OK to start testing from the param server
    while(ros::ok())
    {
        if(nh_ptr->getParam("/wobbler_integration_test/start_testing", dummy))
        {
            break;
        }
    }

    // Set what tests we want to run
    bool test_hokuyo = false;
    bool test_wobbler_transformer = true;

    // Move into formal testing procedure
    ROS_INFO("Starting testing data and point cloud stack");
    ROS_INFO("Testing hokuyo_node");

    if(test_hokuyo == true)
    {
        total_tests++;
        std::string hokuyo_scan_name;
        if(!nh_ptr->getParam("/wobbler_integration_test/hokuyo_scan_name", hokuyo_scan_name))
        {
            ROS_WARN("Failed to get wobbler hokuyo scan topic name parameter!");
        }
        else
        {
            total_tests++;
            tests_passed++;
            SubscriptionVerifier hokuyoVerifier;

            ros::Subscriber hokuyo_sub = nh_ptr->subscribe(hokuyo_scan_name, 1, &SubscriptionVerifier::verifyScan, &hokuyoVerifier);

            if(!hokuyoVerifier.checkSubscription())
            {
                ROS_WARN("Failed to verify correct hokuyo data!");
            }
        }     
    }

    if(test_wobbler_transformer == true)
    {
        total_tests++;
        std::string transformed_scan_cloud_name;
        if(!nh_ptr->getParam("/wobbler_integration_test/transformed_scan_cloud_name", transformed_scan_cloud_name))
        {
            ROS_WARN("Failed to get wobbler transformer scan cloud topic name parameter!");
        }
        else
        {
            total_tests++;
            tests_passed++;
            SubscriptionVerifier transformerVerifier;

            ros::Subscriber hokuyo_sub = nh_ptr->subscribe(transformed_scan_cloud_name, 1, &SubscriptionVerifier::verifyScan, &transformerVerifier);

            if(!transformerVerifier.checkSubscription())
            {
                ROS_WARN("Failed to verify correct wobbler transformer data!");
            }
        }     
    }

    if(test_hokuyo == true)
    {
        //runTest(...);
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
