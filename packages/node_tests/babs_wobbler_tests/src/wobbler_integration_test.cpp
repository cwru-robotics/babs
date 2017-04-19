// Node-level integration testing for the BABS Wobbler
// Created Mar 22 2017 by Trent Ziemer
// Last updated Mar 24 by Trent Ziemer

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Int16.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::NodeHandle * nh_ptr;

// An object that will check if a given topic name with a given type is posting data validly
class SubscriptionVerifier
{
public:
	// Initialize data to default values
    SubscriptionVerifier(std::string, std::string);

    // These three functions are the callbacks called on topic reception
    void verifyScan(const sensor_msgs::LaserScan::ConstPtr&);
    void verifyCloud(const PointCloud::ConstPtr&);
    void verifyInt16(const std_msgs::Int16&);

    // Run this when you're ready to subscribe and wait and validate (any of) the topics given
    void test();

    std::string topic_name;
    std::string param_name;

    // The count for each test, to report to user
	int tests_ran;
	int tests_failed;

private:
	// How long to wait before reporting test failure for topic reception
    int time_to_wait;
    // Success or failure of each topic reception
    bool scan_received;
    bool cloud_received;
    bool int_received;
    // Below are currently unused
    bool scan_verified;
    bool cloud_verified;
    bool int_verified;

    // Generic subscription helper function
    bool checkSubscription();

    // The code name for what verifier to use
    std::string verifier_name;
};

SubscriptionVerifier::SubscriptionVerifier(std::string name, std::string callback_name)
{
	param_name = name;
    time_to_wait = 7000; // milliseconds
    scan_verified = false;
    cloud_verified = false;
    int_verified = false;
    // Below are currently unused
    scan_received = false;
    cloud_received = false;
    int_received = false;

    tests_ran = 0;
    tests_failed = 0;
    verifier_name = callback_name;
}

void SubscriptionVerifier::verifyScan(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	scan_received = true;
	// Check that the scan size is non zero to verify that we received something properly formed
    if((scan_in->ranges).size() > 0)
    {
        scan_verified = true;
    }
}

void SubscriptionVerifier::verifyCloud(const PointCloud::ConstPtr& point_cloud)
{
	cloud_received = true;
	// Check that the point cloud is non zero to verify that we received something properly formed
    if((point_cloud->height * point_cloud->width) > 0)
    {
    	cloud_verified = true;
    }
}

void SubscriptionVerifier::verifyInt16(const std_msgs::Int16& integer) 
{
	int_received = true;
	// Check that the integer is in the range we expected
	//ROS_INFO("RECEIVED VERIFY INT 16 CB");
    if(integer.data > 1 && integer.data < 2300)
    {
    	//ROS_INFO("INT VERIFIED 16 IS  s TRUE");
        int_verified = true;
    }
}

// Call then to actually start the verifier test, assuming proper initialization has happened
void SubscriptionVerifier::test()
{
   	tests_ran++;
    if(!nh_ptr->getParam(param_name, topic_name))
    {
    	tests_failed++;
        ROS_WARN("TEST FAILED: Could not get front stitchers point cloud topic name parameter!");
    }
    else
    {
        tests_ran++;

        // This is SO bad. new w/ no delete afterwards.
		if(verifier_name == "int16")
		{
			ros::Subscriber * subscription_ptr = new ros::Subscriber(nh_ptr->subscribe(topic_name, 1, &SubscriptionVerifier::verifyInt16, this));
		}
		else if (verifier_name == "scan")
		{
			ros::Subscriber * subscription_ptr = new ros::Subscriber(nh_ptr->subscribe(topic_name, 1, &SubscriptionVerifier::verifyScan, this));
		}
		else if (verifier_name == "cloud")
		{
			ros::Subscriber * subscription_ptr = new ros::Subscriber(nh_ptr->subscribe(topic_name, 1, &SubscriptionVerifier::verifyCloud, this));
		}
		else
		{
			ROS_WARN("VERIFIER NAME MATCHED NO EXISTING CALLBACK VERIFIER TYPES");
		}

		// Wait for the subscription to come through (any of them)
        if(!checkSubscription())
        {
        	tests_failed++;
            ROS_WARN("TEST FAILED: Could not verify %s as a posting topic!", topic_name.c_str());
        }
    } 
}

// Generic helper function for hacky subscription checking
bool SubscriptionVerifier::checkSubscription()
{
    int count = 0;
    ros::Rate count_rate(1000); // milliseconds
    while(count < time_to_wait)
    {
        if(scan_verified)
        {
        	scan_verified = false;
            return true;
        }
        else if (cloud_verified)
        {
        	cloud_verified = false;
        	return true;
        }
        else if(int_verified)
        {
        	int_verified = false;
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
    ros::init(argc, argv, "wobbler_integration_test");
    ros::NodeHandle nh;
    nh_ptr = &nh;

    // Keep track of how many tests fail, out of the total number of tests ran
    int tests_failed = 0;
    int total_tests = 0;

    double dummy;

    // Wait for us to get the OK to start running tests from the param server
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
    bool test_stitchers = true;

    bool test_motors = true;

    // Move into formal testing procedure
    ROS_INFO("Starting testing data and point cloud stack!");

    if(test_hokuyo == true)
    {
        ROS_INFO("Testing hokuyo_node...");
    	SubscriptionVerifier verifier1("/wobbler_integration_test/front_hokuyo_scan_name", "scan");
    	verifier1.test();
    	total_tests += verifier1.tests_ran;
    	tests_failed += verifier1.tests_failed;

    	SubscriptionVerifier verifier2("/wobbler_integration_test/rear_hokuyo_scan_name", "scan");
    	verifier2.test();
    	total_tests += verifier2.tests_ran;
    	tests_failed += verifier2.tests_failed;
    }

    if(test_wobbler_transformer == true)
    {
    	ROS_INFO("Testing wobbler transformers...");
    	SubscriptionVerifier verifier1("/wobbler_integration_test/front_transformed_scan_cloud_name", "cloud");
    	verifier1.test();
    	total_tests += verifier1.tests_ran;
    	tests_failed += verifier1.tests_failed;

    	SubscriptionVerifier verifier2("/wobbler_integration_test/rear_transformed_scan_cloud_name", "cloud");
    	verifier2.test();
    	total_tests += verifier2.tests_ran;
    	tests_failed += verifier2.tests_failed;
    }
    
    
    if(test_stitchers == true)
    {
    	ROS_INFO("Testing pcl stitchers...");
    	SubscriptionVerifier verifier1("/wobbler_integration_test/front_stitched_cloud_name", "cloud");
    	verifier1.test();
    	total_tests += verifier1.tests_ran;
    	tests_failed += verifier1.tests_failed;

    	SubscriptionVerifier verifier2("/wobbler_integration_test/rear_stitched_cloud_name", "cloud");
    	verifier2.test();
    	total_tests += verifier2.tests_ran;
    	tests_failed += verifier2.tests_failed;
    }

    if(test_motors == true)
    {
    	ROS_INFO("Testing dynamixel motor cmd and angle topics...");
    	SubscriptionVerifier verifier1("/wobbler_integration_test/front_motor_angle_name", "int16");
    	verifier1.test();
    	total_tests += verifier1.tests_ran;
    	tests_failed += verifier1.tests_failed;

    	SubscriptionVerifier verifier2("/wobbler_integration_test/rear_motor_angle_name", "int16");
    	verifier2.test();
    	total_tests += verifier2.tests_ran;
    	tests_failed += verifier2.tests_failed;

    	SubscriptionVerifier verifier3("/wobbler_integration_test/front_motor_command_name", "int16");
    	verifier3.test();
    	total_tests += verifier3.tests_ran;
    	tests_failed += verifier3.tests_failed;

    	SubscriptionVerifier verifier4("/wobbler_integration_test/rear_motor_command_name", "int16");
    	verifier4.test();
    	total_tests += verifier4.tests_ran;
    	tests_failed += verifier4.tests_failed;
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
