#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2 
#include <pcl/common/common_headers.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>

// basic file operations
#include <fstream>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <string.h>
#include <stdio.h>  
#include <std_msgs/Float64.h>

#include <sstream>
#include <boost/filesystem.hpp>
#include <vector>



using namespace std;

bool shouldCapture;
bool shouldSpawn;
bool outputPositive;
int fileCounter;



ros::Publisher cropped_cloud_pub;

struct Quaternion {
	double w;
	double x;
	double y;
	double z;
};

//save as RAW
void saveImage(int **imageMatrix, int xdim, int ydim, char valuesFile[], char imageFile[]) {

	boost::filesystem::create_directories("./NEGATIVE_OUTPUT");
	boost::filesystem::create_directories("./POSITIVE_OUTPUT");


	ofstream heightfile(valuesFile, ios::out | ios::binary);
	FILE *image = fopen(imageFile, "wb");

	for (unsigned int yIndex = 0; yIndex < ydim; yIndex++)
	{
		for (unsigned int xIndex = 0; xIndex < xdim; xIndex++)
		{
			//write the values to a separate file so we can see the numbers
			if(xIndex == 0) {
				heightfile << "\n";
			}
			heightfile << imageMatrix[xIndex][yIndex] << ", ";

			//now write the actual image
			fwrite(&imageMatrix[xIndex][yIndex], sizeof(unsigned char), 1, image);
		}
	}
	heightfile.close();
	fclose(image); 
	ROS_INFO("done writing image");
}

//iterate through maxInterpolations times and each time fill in any pixels with the unknown value (255)
//that are bordered by 4 or more known pixels by taking their average 
void interpolateMissingData(int **imageMatrix, int xdim, int ydim, int maxInterpolations) {
	for (int i = 0; i < maxInterpolations; i++) {
		for (unsigned int xIndex = 0; xIndex < xdim; xIndex++)
		{	
			for (unsigned int yIndex = 0; yIndex < ydim; yIndex++)
			{					
				//see if we cell heighty is unknown (value would be 255)
				if (imageMatrix[xIndex][yIndex] == 255) {
					int surroundingCells[8];

					//to ensure we don't try to access any pixel indicies outside of the image
					//when we look at surrounding pixels. Set any outside the boundaries to 255
					if(xIndex == xdim-1) {
						surroundingCells[2] = 255;
						surroundingCells[4] = 255;
						surroundingCells[7] = 255;
					}
					if(xIndex == 0) {
						surroundingCells[0] = 255;
						surroundingCells[3] = 255;
						surroundingCells[5] = 255;
					}
					if(yIndex == ydim-1) {
						surroundingCells[5] = 255;
						surroundingCells[6] = 255;
						surroundingCells[7] = 255;
					}
					if(yIndex == 0) {
						surroundingCells[0] = 255;
						surroundingCells[1] = 255;
						surroundingCells[2] = 255;
					}

					if (surroundingCells[0] != 255) {
						surroundingCells[0] = imageMatrix[xIndex-1][yIndex-1];
					}
					if (surroundingCells[1] != 255) {
						surroundingCells[1] = imageMatrix[xIndex][yIndex-1];

					}
					if (surroundingCells[2] != 255) {
						surroundingCells[2] = imageMatrix[xIndex+1][yIndex-1];

					}
					if (surroundingCells[3] != 255) {
						surroundingCells[3] = imageMatrix[xIndex-1][yIndex];

					}
					if (surroundingCells[4] != 255) {
						surroundingCells[4] = imageMatrix[xIndex+1][yIndex];

					}
					if (surroundingCells[5] != 255) {
						surroundingCells[5] = imageMatrix[xIndex-1][yIndex+1];

					}
					if (surroundingCells[6] != 255) {
						surroundingCells[6] = imageMatrix[xIndex][yIndex+1];

					}
					if (surroundingCells[7] != 255) {
						surroundingCells[7] = imageMatrix[xIndex+1][yIndex+1];

					}

					int knownPixelValueSum = 0;
					int numKnown = 0;
					for (int j = 0; j < 8; j++) {
						if (surroundingCells[j] != 255) {
							numKnown++;
							knownPixelValueSum += surroundingCells[j];
						}
					}
					if (numKnown >= 4) {
						imageMatrix[xIndex][yIndex] = knownPixelValueSum/numKnown;
					}
				}
			}
		}
	}
}

void take_snapshot(const sensor_msgs::PointCloud2& input) {

	ROS_INFO("Taking snapshot");

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg(input,cloud);

	int cloudsize = (cloud.width) * (cloud.height);

	//for debug purposes
	double maxX = -100;
	double maxY = -100;
	double maxZ = -100;
	double minX = 100;
	double minY = 100;
	double minZ = 100;

	//we're interesting only in the 1m x 1m square in front of the robot
	double xMin = -0.5;
	double xMax = 0.5;
	double yMin = 0.5;
	double yMax = 1.5;

	//and we need to limit the height range
	//height range should be be centered on ground instead of the height of the wobbler,
	//wobbler is about 1 meter of the ground (but wobbler height is considered height 0)
	double zMin = -3;
	double zMax = 1;

	//1 meter square will have a resolution of 5cm, so its 20x20 pixels (400 total pixels)
	int xdim = 20;
	int ydim = 20;
	double intensities[xdim*ydim];

	//initialize with nan
	for (int i=0; i < xdim*ydim; i++) {
		intensities[i] = NAN;
	}

	for (int i=0; i< cloudsize; i++) {

		//TODO tf to gravity frame

		//in the gazebo's frame the point cloud's y is in the -z direction,
		//z is in the y direction, and x is the same
		double x = cloud[i].x;
		double y = cloud[i].z;
		double z = -cloud[i].y;


		//only care about points that fall in our square
		if (xMin < x && xMax > x &&
				yMin < y && yMax > y) {

			//for debug purposes
			if (x < minX) {
				minX = x;
			}
			if (y < minY) {
				minY = y;
			}
			if (z < minZ) {
				minZ = z;
			}
			if (x > maxX) {
				maxX = x;
			}
			if (y > maxY) {
				maxY = y;
			}
			if (z > maxZ) {
				maxZ = z;
			}

			double fraction = (x - xMin)/(xMax - xMin);
			//round by adding 0.5 and casting (truncating)
			int xIndex = (int)((xdim-1) * fraction + 0.5);

			fraction = (y - yMin)/(yMax - yMin);
			int yIndex = (int)((ydim-1) * fraction + 0.5);

			int index = yIndex + xIndex * ydim;

			fraction = (z - zMin)/(zMax - zMin);
			if(fraction > 1) {
				fraction = 1;
			}

			//scale fall in range 0-199
			//255 is reserved for unknown pixels, the gap 200-254 may help the 
			//future neural net more easily differentiate unknowns from other values
			double value = fraction * 199;

			//save the pixel as the highest point we come across
			if (isnan(intensities[index]) || intensities[index] < value) {
				intensities[index] = value;
			}
		}
		else {
			//otherwise we don't want these points, set them to NAN
			cloud[i].x = NAN;
			cloud[i].z = NAN;
			cloud[i].y = NAN;
		}
	}

	//debug purposes, so we can see the range of our square in all 3 dimensions
	ofstream bounds("bounds.txt", ios::out | ios::binary);
	bounds << minX << "\n";
	bounds << maxX << "\n";
	bounds << minY << "\n";
	bounds << maxY << "\n";
	bounds << minZ << "\n";
	bounds << maxZ << "\n";
	bounds.close();

	//convert intensities array of doubles to ints for the imageMatrix
	int **imageMatrix;
	imageMatrix = new int*[xdim];

	for (unsigned int xIndex = 0; xIndex < xdim; xIndex++)
	{	
		imageMatrix[xIndex] = new int[ydim];
		for (unsigned int yIndex = 0; yIndex < ydim; yIndex++)
		{	
			int index = xIndex + yIndex * xdim;
			if(isnan(intensities[index])) {
				//anything still nan is unknown value so set it to 255 to separate it from the 0-200 range
				imageMatrix[xIndex][yIndex] = 255;
			}
			else {
				imageMatrix[xIndex][yIndex] = (int)(intensities[index] + 0.5);
			}
		}
	}

	ostringstream ss;
	ss << fileCounter;
	string str = ss.str();
	char suffix[str.length()];
	strcpy (suffix,str.c_str());	

	char valuesFile[strlen(suffix) + strlen("./********_OUTPUT/values.txt")];
	char imageFile[strlen(suffix) + strlen("./********_OUTPUT/heightmap.data")];
	if(outputPositive) {
		strcpy (valuesFile,"./POSITIVE_OUTPUT/values");	
		strcpy (imageFile,"./POSITIVE_OUTPUT/heightmap");	
	}
	else {
		strcpy (valuesFile,"./NEGATIVE_OUTPUT/values");
		strcpy (imageFile,"./NEGATIVE_OUTPUT/heightmap");	
	}

	strncat(valuesFile, suffix, strlen(suffix));
	strncat(valuesFile, ".txt", strlen(".txt"));
	strncat(imageFile, suffix, strlen(suffix));
	strncat(imageFile, ".data", strlen(".data"));

	//write to disk
	saveImage(imageMatrix, xdim, ydim, valuesFile, imageFile);
	fileCounter++;







	//TODO decide if ya wanna use interpolation







	//allow a max of 3 passes of interpolation, filling in small holes in the data, but not guessing at
	//large, unknown areas

	// interpolateMissingData(imageMatrix, xdim, ydim, 3);
	// char valuesInterpolatedFile[] = "valuesInterpolated.txt";
	// char heightmapInterpolatedFile[] = "heightmapInterpolated.data";
	// saveImage(imageMatrix, xdim, ydim, valuesInterpolatedFile, heightmapInterpolatedFile);

	sensor_msgs::PointCloud2 croppedCloud;
	pcl::toROSMsg(cloud, croppedCloud);

	if(!cropped_cloud_pub) {
		ROS_WARN("Publisher invalid!");
	}
	else {
		cropped_cloud_pub.publish(croppedCloud);
	}
}


void pointCloudCallback(const sensor_msgs::PointCloud2& cloud) {

	//only take_one_snapshot each time the node is run
	if (shouldCapture) {
		take_snapshot(cloud);
		shouldCapture = false;
		shouldSpawn = true;
	}
}

//helper math function
void toEulerianAngle(const Quaternion& q, double& pitch, double& roll, double& yaw)
{
	double ysqr = q.y * q.y;
	double t0 = -2.0f * (ysqr + q.z * q.z) + 1.0f;
	double t1 = +2.0f * (q.x * q.y - q.w * q.z);
	double t2 = -2.0f * (q.x * q.z + q.w * q.y);
	double t3 = +2.0f * (q.y * q.z - q.w * q.x);
	double t4 = -2.0f * (q.x * q.x + ysqr) + 1.0f;

	t2 = t2 > 1.0f ? 1.0f : t2;
	t2 = t2 < -1.0f ? -1.0f : t2;

	pitch = std::asin(t2);
	roll = std::atan2(t3, t4);
	yaw = std::atan2(t1, t0);
}

//helper math function
Quaternion toQuaternion(double pitch, double roll, double yaw)
{
	Quaternion q;
	double t0 = std::cos(yaw * 0.5f);
	double t1 = std::sin(yaw * 0.5f);
	double t2 = std::cos(roll * 0.5f);
	double t3 = std::sin(roll * 0.5f);
	double t4 = std::cos(pitch * 0.5f);
	double t5 = std::sin(pitch * 0.5f);

	q.w = t0 * t2 * t4 + t1 * t3 * t5;
	q.x = t0 * t3 * t4 - t1 * t2 * t5;
	q.y = t0 * t2 * t5 + t1 * t3 * t4;
	q.z = t1 * t2 * t4 - t0 * t3 * t5;
	return q;
}

int main(int argc, char** argv){

	ros::init(argc, argv, "data_collection");
	shouldCapture = false;
	shouldSpawn = true;
	ROS_INFO("running data collection loop!");
	fileCounter = 0;

	//TODO future work: spawn in random spot
	//check accerometer, speed, collisions to see if in a valid pose (aka upright, not colliding)
	//save point cloud
	//attempt to drive a meter or something, while checking for tipping, failure to progress, collision, etc 
	//determine whether successful, saved pt cloud success label together as a pair

	ros::NodeHandle n;
	cropped_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/cropped_cloud", 1);
	ros::Subscriber pcl_subscriber = n.subscribe("/kinect/depth/points", 1, pointCloudCallback);

	ros::Duration half_sec(0.5);
	bool service_ready = false;
	while (!service_ready) {
		service_ready = ros::service::exists("/gazebo/set_model_state",true);
		ROS_INFO("waiting for set_model_state service");
		half_sec.sleep();
	}
	ROS_INFO("set_model_state service exists");
	ros::ServiceClient set_model_state_client = 
			n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

	vector<string> objects;
	//indexes 0-2 are a no go
	objects.push_back("cinder_block_2");
	objects.push_back("cardboard_box");
	objects.push_back("cinder_block");

	//indexes 3-6 are coo
	objects.push_back("monkey wrench");
	objects.push_back("t_brace_part");
	objects.push_back("cross_joint_part");
	//index 6 shouldn't be rotated
	objects.push_back("grey_wall_0");

	while(true) {
		ros::spinOnce();

		if (shouldSpawn) {
			shouldSpawn = false;

			int item_index = (int)(8 * ((double) rand() / (RAND_MAX)));
			ROS_INFO("item_index is %d", item_index);
			bool no_rot = false;

			if(item_index <=2) {
				outputPositive = false;
			}
			else {
				outputPositive = true;
				if(item_index >= 6) {
					no_rot = true;
				}
			}

			if (item_index == 7) {
				//use ground plane instead of object so don't need to change location
			}
			else {
				string item = objects.at(item_index);

				char out[item.length()];
				strcpy (out,item.c_str());	
				ROS_INFO("item is %s", out);

				gazebo_msgs::SetModelState model_state_srv_msg;
				model_state_srv_msg.request.model_state.model_name = item;			

				//convert quat to roll pitch yaw and back
				//			Quaternion q;
				//			q.x = model_state_srv_msg.request.model_state.pose.orientation.x;
				//			q.y = model_state_srv_msg.request.model_state.pose.orientation.y;
				//			q.z = model_state_srv_msg.request.model_state.pose.orientation.z;
				//			q.w = model_state_srv_msg.request.model_state.pose.orientation.w;
				//			double pitch, roll, yaw;
				//			toEulerianAngle( q, &pitch, &roll, &yaw);

				double pitch, roll, yaw;


				if(no_rot) {
					pitch = 0;
					roll = -1.37;
					yaw = -1.57;
				}
				else {
					pitch = 0;
					roll = 0;
					yaw = ((double) rand() / (RAND_MAX))*2*3.14159;
				}
				Quaternion q = toQuaternion(pitch,roll,yaw);

				model_state_srv_msg.request.model_state.pose.orientation.x = q.x;
				model_state_srv_msg.request.model_state.pose.orientation.y = q.y;
				model_state_srv_msg.request.model_state.pose.orientation.z = q.z;
				model_state_srv_msg.request.model_state.pose.orientation.w = q.w;

				double xDisplacement;
				double yDisplacement;
				xDisplacement = ((double) rand() / (RAND_MAX)) - 0.5;
				if (no_rot) {
					model_state_srv_msg.request.model_state.pose.position.x = 0.9 + xDisplacement;
					model_state_srv_msg.request.model_state.pose.position.y = 0.0;
					model_state_srv_msg.request.model_state.pose.position.z = -0.1;
				}
				else {
					yDisplacement = ((double) rand() / (RAND_MAX)) - 0.5;
					model_state_srv_msg.request.model_state.pose.position.x = 1.5 + 0.75*xDisplacement;
					model_state_srv_msg.request.model_state.pose.position.y = 0.0 + yDisplacement;
					model_state_srv_msg.request.model_state.pose.position.z = 0.5;
				}

				model_state_srv_msg.request.model_state.twist.linear.x= 0.0; //2cm/sec
				model_state_srv_msg.request.model_state.twist.linear.y= 0.0;
				model_state_srv_msg.request.model_state.twist.linear.z= 0.0;
				model_state_srv_msg.request.model_state.twist.angular.x= 0.0;
				model_state_srv_msg.request.model_state.twist.angular.y= 0.0;
				model_state_srv_msg.request.model_state.twist.angular.z= 0.0;

				model_state_srv_msg.request.model_state.reference_frame = "world";

				set_model_state_client.call(model_state_srv_msg);
				bool result = model_state_srv_msg.response.success;
				if (!result)
					ROS_WARN("service call to set_model_state failed!");
			}

			//now wait for the object to drop/settle and take a pic
			half_sec.sleep();
			shouldCapture = true;
			//wait until capture function to be done
			while(shouldCapture == true) {
				ros::spinOnce();
				half_sec.sleep();
			}
			//now remove the item if there was one
			if (item_index == 7) {
				//ground plane don't need to move anything
			}
			else {
				string item = objects.at(item_index);
				gazebo_msgs::SetModelState model_state_srv_msg;
				model_state_srv_msg.request.model_state.model_name = item;			

				model_state_srv_msg.request.model_state.pose.position.x = 0.0;
				model_state_srv_msg.request.model_state.pose.position.y = 3.0;
				model_state_srv_msg.request.model_state.pose.position.z = 2.0;

				double pitch, roll, yaw;
				pitch = 0;
				roll = 0;
				yaw = 0;
				Quaternion q = toQuaternion(pitch,roll,yaw);
				model_state_srv_msg.request.model_state.pose.orientation.x = q.x;
				model_state_srv_msg.request.model_state.pose.orientation.y = q.y;
				model_state_srv_msg.request.model_state.pose.orientation.z = q.z;
				model_state_srv_msg.request.model_state.pose.orientation.w = q.w;

				model_state_srv_msg.request.model_state.reference_frame = "world";

				set_model_state_client.call(model_state_srv_msg);
				bool result = model_state_srv_msg.response.success;
				if (!result)
					ROS_WARN("service call to set_model_state failed2!");
			}

		}
	}
	//	ros::spin();
	return 0;
}










































