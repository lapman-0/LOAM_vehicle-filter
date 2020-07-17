#include <math.h>
#include <vector>
#include <aloam_velodyne/common.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "lidarFactor.hpp"
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"

ros::Publisher pub;
double ofsx = 0;
double ofsy = 0; 
double ofsz = 0;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	//const float thres = 1.9;
  	// Container for original & filtered data 
 	//pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  	//pcl::PCLPointCloud2 cloud_filtered;
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);


  	// Convert to PCL data type
  	pcl::fromROSMsg(*cloud_msg, *cloud);

  	// identify bad points
  	//*cloud_filtered = *cloud;
  	
  	const float bad_point = std::numeric_limits<float>::quiet_NaN();
  	//ROS_INFO("cloud x = %f , y = %f z = %f",ofsx , ofsy , ofsz);


	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		float distance = (sqrt((cloud->points[i].x - ofsx)*(cloud->points[i].x - ofsx) + 
    					   	   (cloud->points[i].y - ofsy)*(cloud->points[i].y - ofsy) + 
    				   		   (cloud->points[i].z - ofsz)*(cloud->points[i].z - ofsz)));
		if (distance < 2.5)
		{
        	cloud->points[i].x = bad_point;
        	cloud->points[i].y = bad_point;
        	cloud->points[i].z = bad_point;
      	}
    }

    //remove NaN points
    boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
	pcl::removeNaNFromPointCloud(*cloud, *indices);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(indices);
	extract.setNegative(false);
	extract.filter(*cloud_filtered);

	/*
	pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(-1.5, -1, -1.5, 1.0));
    boxFilter.setMax(Eigen::Vector4f(1.5, 1.5, 1.5, 1.0));

    boxFilter.setInputCloud(cloud);
    boxFilter.setNegative(true);
    boxFilter.filter(*cloud_filtered);
    */
    
    
    
  	// Convert to ROS data type
  	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*cloud_filtered, output);

  	// Publish the data
  	pub.publish (output);
}

int
main (int argc, char** argv)
{
	// Initialize ROS
  	ros::init (argc, argv, "bot_rm");
  	ros::NodeHandle nh;

  	// Create a ROS subscriber for the input point cloud
  	ros::Subscriber sub = nh.subscribe ("velodyne_points", 1000, cloud_cb);

  	// Create a ROS publisher for the output point cloud
  	pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_rmed", 1000);

  	// Spin
  	ros::spin ();
}