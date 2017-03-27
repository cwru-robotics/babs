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
    time_to_wait = 2000; // milliseconds
    scan_verified = false;
    cloud_verified = false;
    // Currently unused, below
    scan_received = false;
    cloud_received = false;
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

    // Keep track of how many tests fail, out of the total number of tests ran
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
    bool test_stitchers = true;


    // Move into formal testing procedure
    ROS_INFO("Starting testing data and point cloud stack!");
    ROS_INFO("Testing hokuyo_node...");

    if(test_hokuyo == true)
    {
    	// Front
        total_tests++;
        std::string hokuyo_scan_name;
        SubscriptionVerifier hokuyoVerifier;
        if(!nh_ptr->getParam("/wobbler_integration_test/front_hokuyo_scan_name", hokuyo_scan_name))
        {
        	tests_failed++;
            ROS_WARN("TEST FAILED: Could not verify front wobbler hokuyo scan topic name parameter!");
        }
        else
        {
            total_tests++;

            ros::Subscriber hokuyo_sub = nh_ptr->subscribe(hokuyo_scan_name, 1, &SubscriptionVerifier::verifyScan, &hokuyoVerifier);

            if(!hokuyoVerifier.checkSubscription())
            {
            	tests_failed++;
                ROS_WARN("TEST FAILED: Could not verify valid front hokuyo data!");
            }
        }

        // Rear  
        total_tests++;
        if(!nh_ptr->getParam("/wobbler_integration_test/rear_hokuyo_scan_name", hokuyo_scan_name))
        {
        	tests_failed++;
            ROS_WARN("TEST FAILED: Could not verify rear wobbler hokuyo scan topic name parameter!");
        }
        else
        {
            total_tests++;

            ros::Subscriber hokuyo_sub = nh_ptr->subscribe(hokuyo_scan_name, 1, &SubscriptionVerifier::verifyScan, &hokuyoVerifier);

            if(!hokuyoVerifier.checkSubscription())
            {
            	tests_failed++;
                ROS_WARN("TEST FAILED: Could not verify valid rear hokuyo data!");
            }
        }  
    }

    ROS_INFO("Testing wobbler transformers...");
    if(test_wobbler_transformer == true)
    {
    	// Front
        total_tests++;
        std::string transformed_scan_cloud_name;
        SubscriptionVerifier transformerVerifier;
        if(!nh_ptr->getParam("/wobbler_integration_test/front_transformed_scan_cloud_name", transformed_scan_cloud_name))
        {
        	tests_failed++;
            ROS_WARN("TEST FAILED: Could not get front wobbler transformer scan cloud topic name parameter!");
        }
        else
        {
            total_tests++;

            ros::Subscriber transformer_sub = nh_ptr->subscribe(transformed_scan_cloud_name, 1, &SubscriptionVerifier::verifyCloud, &transformerVerifier);

            if(!transformerVerifier.checkSubscription())
            {
            	tests_failed++;
                ROS_WARN("TEST FAILED: Could not verify valid front wobbler transformer data!");
            }
        }     

        // Rear
        if(!nh_ptr->getParam("/wobbler_integration_test/rear_transformed_scan_cloud_name", transformed_scan_cloud_name))
        {
        	tests_failed++;
            ROS_WARN("TEST FAILED: Could not get rear wobbler transformer scan cloud topic name parameter!");
        }
        else
        {
            total_tests++;

            ros::Subscriber transformer_sub = nh_ptr->subscribe(transformed_scan_cloud_name, 1, &SubscriptionVerifier::verifyCloud, &transformerVerifier);

            if(!transformerVerifier.checkSubscription())
            {
            	tests_failed++;
                ROS_WARN("TEST FAILED: Could not verify valid rear wobbler transformer data!");
            }
        } 
    }

    ROS_INFO("Testing pcl stitchers...");
    if(test_stitchers == true)
    {
    	// Front
        total_tests++;
        std::string stitched_cloud_name;
        SubscriptionVerifier stitcherVerifier;
        if(!nh_ptr->getParam("/wobbler_integration_test/front_stitched_cloud_name", stitched_cloud_name))
        {
        	tests_failed++;
            ROS_WARN("TEST FAILED: Could not get front stitchers point cloud topic name parameter!");
        }
        else
        {
            total_tests++;

            ros::Subscriber stitcher_sub = nh_ptr->subscribe(stitched_cloud_name, 1, &SubscriptionVerifier::verifyCloud, &stitcherVerifier);

            if(!stitcherVerifier.checkSubscription())
            {
            	tests_failed++;
                ROS_WARN("TEST FAILED: Could not verify valid front pcl stitcher data!");
            }
        } 

        // Rear    
        if(!nh_ptr->getParam("/wobbler_integration_test/rear_stitched_cloud_name", stitched_cloud_name))
        {
        	tests_failed++;
            ROS_WARN("TEST FAILED: Could not get rear stitchers point cloud topic name parameter!");
        }
        else
        {
            total_tests++;

            ros::Subscriber stitcher_sub = nh_ptr->subscribe(stitched_cloud_name, 1, &SubscriptionVerifier::verifyCloud, &stitcherVerifier);

            if(!stitcherVerifier.checkSubscription())
            {
            	tests_failed++;
                ROS_WARN("TEST FAILED: Could not verify valid rear pcl stitcher data!");
            }
        } 
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
