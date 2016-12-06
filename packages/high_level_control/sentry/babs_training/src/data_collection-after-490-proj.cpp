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

using namespace std;

bool published;

ros::Publisher cropped_cloud_pub;

//save as RAW
void saveImage(int **imageMatrix, int xdim, int ydim, char valuesFile[], char imageFile[]) {
	//	
	//	char valuesFile[] = "vals.txt";
	//	char imageFile[] = "img.data"; 

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
	ROS_INFO("width: %d", cloud.width);
	ROS_INFO("height: %d", cloud.height);

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

	char valuesFile[] = "values.txt";
	char imageFile[] = "heightmap.data";

	//write to disk
	saveImage(imageMatrix, xdim, ydim, valuesFile, imageFile);

	//allow a max of 3 passes of interpolation, filling in small holes in the data, but not guessing at
	//large, unknown areas
	interpolateMissingData(imageMatrix, xdim, ydim, 3);

	char valuesInterpolatedFile[] = "valuesInterpolated.txt";
	char heightmapInterpolatedFile[] = "heightmapInterpolated.data";
	saveImage(imageMatrix, xdim, ydim, valuesInterpolatedFile, heightmapInterpolatedFile);

	sensor_msgs::PointCloud2 croppedCloud;
	pcl::toROSMsg(cloud, croppedCloud);

	if(!cropped_cloud_pub) {
		ROS_WARN("Publisher invalid!");
	}
	else {
		cropped_cloud_pub.publish(croppedCloud);
		ROS_INFO("Publisher VALID!!!!");
	}
}


void pointCloudCallback(const sensor_msgs::PointCloud2& cloud) {

	//only take_one_snapshot each time the node is run
	if (!published) {
		take_snapshot(cloud);
		published = true;
	}
}

int main(int argc, char** argv){

	ros::init(argc, argv, "data_collection");

	published = false;

	ROS_INFO("running data collection loop!");

	//TODO future work: spawn in random spot
	//check accerometer, speed, collisions to see if in a valid pose (aka upright, not colliding)
	//save point cloud
	//attempt to drive a meter or something, while checking for tipping, failure to progress, collision, etc 
	//determine whether successful, saved pt cloud success label together as a pair

	ros::NodeHandle n;

	cropped_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/cropped_cloud", 1);

	ros::Subscriber pcl_subscriber = n.subscribe("/kinect/depth/points", 1, pointCloudCallback);

	ros::spin();
	return 0;
}
