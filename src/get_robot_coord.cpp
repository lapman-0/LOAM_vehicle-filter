#include <math.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <std_msgs/Float32MultiArray.h>

//ros::Publisher pub;

int main (int argc, char** argv)
{
	// Initialize ROS
  	ros::init (argc, argv, "get_robot_coord");
  	ros::NodeHandle nh;
  	ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("coord" , 50);


  	tf::TransformListener listener;
  	double x, y, z;
  	ros::Rate rate(10);
  	while (nh.ok()) {
  		tf::StampedTransform transform;

		try
  		{
  			ros::Time now = ros::Time::now();
      	listener.waitForTransform("/camera_init", "/aft_mapped", ros::Time(0), ros::Duration(10));
    		listener.lookupTransform("/camera_init", "/aft_mapped", ros::Time(0) , transform);
      	x = transform.getOrigin().x();
      	y = transform.getOrigin().y();
  			z = transform.getOrigin().z();
  			//printf("x=%f y=%f z=%f xxxxxxxxxxxxxxxxxxxxxxx\n", x, y, z);
  			std_msgs::Float32MultiArray coordinates;
  			coordinates.data.resize(3);
  			coordinates.data[0] = x;
  			coordinates.data[1] = y;
  			coordinates.data[2] = z;
  			pub.publish(coordinates);
  			ros::spinOnce();
  			rate.sleep();
  		}
    	catch (tf::TransformException &ex) 
    	{
      		ROS_ERROR("%s",ex.what());
      		ros::Duration(1.0).sleep();
    	}
    }
    return 0;
}