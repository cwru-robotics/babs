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
    bool scan_received;
    bool cloud_received;
};

SubscriptionVerifier::SubscriptionVerifier()
{
    time_to_wait = 3000; // milliseconds
    scan_verified = false;
    cloud_verified = false;
    scan_received = false;
    cloud_received = false;
}

void SubscriptionVerifier::verifyScan(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	        ROS_ERROR("IN SCAN CB");
	scan_received = true;
    if(1)
    {
        scan_verified = true;
    }
}

void SubscriptionVerifier::verifyCloud(const PointCloud::ConstPtr& point_cloud)
{
	        ROS_ERROR("IN CLOUD CB");
	cloud_received = true;
    if(1)
    {

        cloud_verified = true;
    }
}

bool SubscriptionVerifier::checkSubscription()
{
    int count = 0;
    ros::Rate count_rate(1000); // milliseconds
    while(count < time_to_wait)
    {
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
    int tests_failed = 0;
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
    bool test_hokuyo = true;
    bool test_wobbler_transformer = true;

    // Move into formal testing procedure
    ROS_INFO("Starting testing data and point cloud stack");
    ROS_INFO("Testing hokuyo_node...");

    if(test_hokuyo == true)
    {
        total_tests++;
        std::string hokuyo_scan_name;
        if(!nh_ptr->getParam("/wobbler_integration_test/hokuyo_scan_name", hokuyo_scan_name))
        {
        	tests_failed++;
            ROS_WARN("TEST FAILED: Could not verify wobbler hokuyo scan topic name parameter!");
        }
        else
        {
            total_tests++;

            SubscriptionVerifier hokuyoVerifier;

            ros::Subscriber hokuyo_sub = nh_ptr->subscribe(hokuyo_scan_name, 1, &SubscriptionVerifier::verifyScan, &hokuyoVerifier);

            if(!hokuyoVerifier.checkSubscription())
            {
            	tests_failed++;
                ROS_WARN("TEST FAILED: Could not verify correct hokuyo data!");
            }
        }     
    }

    ROS_INFO("Testing wobbler transformers...");
    if(test_wobbler_transformer == true)
    {
        total_tests++;
        std::string transformed_scan_cloud_name;
        if(!nh_ptr->getParam("/wobbler_integration_test/transformed_scan_cloud_name", transformed_scan_cloud_name))
        {
        	tests_failed++;
            ROS_WARN("TEST FAILED: Could not get wobbler transformer scan cloud topic name parameter!");
        }
        else
        {
            total_tests++;
            SubscriptionVerifier transformerVerifier;

            ros::Subscriber hokuyo_sub = nh_ptr->subscribe(transformed_scan_cloud_name, 1, &SubscriptionVerifier::verifyScan, &transformerVerifier);

            if(!transformerVerifier.checkSubscription())
            {
            	tests_failed++;
                ROS_WARN("TEST FAILED: Could not verify correct wobbler transformer data!");
            }
        }     
    }

    if(test_hokuyo == true)
    {
        //runTest(...);
    }

	if(tests_failed == 0)
	{
		ROS_INFO("INTEGRATION TESTING PASSED: %d tests completed.", total_tests);
	}
	else
	{
		ROS_WARN("INTEGRATION TESTING FAILED: %d tests failed out of %d total!", tests_failed, total_tests);
	}


    ROS_INFO("Closing out of tests...");

    return 0;
}
