#include <math.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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

//using namespace sensor_msgs;
//using namespace message_filters;

ros::Publisher pub1 , pub2;
float x,y,z;
float rsize = 4.3;
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_const (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_full (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr frame (new pcl::PointCloud<pcl::PointXYZ>);



double thres = 1.2;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    float x1 , y1 , z1;
    x1 = x;
    y1 = y;
    z1 = z;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_const (new pcl::PointCloud<pcl::PointXYZ>);
  
    pcl::PointCloud<pcl::PointXYZ>::Ptr frame2 (new pcl::PointCloud<pcl::PointXYZ>);
  
    const float bad_point = std::numeric_limits<float>::quiet_NaN();
  
    boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);

    pcl::fromROSMsg(*cloud_msg, *cloud1);
    
    if (cloud0->points.size() == 0) {
        *cloud0 = *cloud1;
        *cloud_full = *cloud1;
        *frame = *cloud1;
        printf("4444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444\n" );
        return;
    }

    else {

        printf("running else////////////////////////////////////////\n");
        printf("c0= %d c1=%d ----------------------------------------\n", cloud1->points.size(), cloud0->points.size());

        if (cloud1->points.size() > cloud0->points.size()) {

            /*for (size_t i = 0; i < (cloud0->points.size()); i++) {
            /*
            cloud1->points[i].x = bad_point;
            cloud1->points[i].y = bad_point;
            cloud1->points[i].z = bad_point;
          
            if (cloud1->points[i] == cloud0->points[i]) {
            cloud1->points[i].x = bad_point;
            cloud1->points[i].y = bad_point;
            cloud1->points[i].z = bad_point;
          
    

            boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
            pcl::removeNaNFromPointCloud(*cloud1, *indices);
            pcl::ExtractIndices<pcl::PointXYZ> removePrevPts;
            removePrevPts.setInputCloud(cloud1);
            removePrevPts.setIndices(indices);
            removePrevPts.setNegative(false);
            removePrevPts.filter(*cloud1);
            //cloud1->points.resize();
            */

      
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;

            float point_radius = 0.0000005;

            //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_reduced (new pcl::PointCloud<pcl::PointXYZ>);
            //cloud_const->reserve(cloud1->points.size());

            if (cloud1->points.size()>1){

                bool *marked= new bool[cloud1->points.size()];
                //memset(marked,false,sizeof(bool)*cloud1->points.size());

                //pcl:: PointCloud<PointXYZ>::Ptr cloud1 (new pcl::PointCloud<PointXYZ>);

                kdtree.setInputCloud (cloud1);
                for (unsigned int c=0; c < cloud0->points.size(); c++) {

                    if(cloud1->points.size()<2){
                        printf("9999999999999999999999999999999999999999999999999999999999999999999999999999999999999999\n" );
                        break;
                    }

                    pcl::PointXYZ searchPoint;

                    searchPoint.x = cloud0->points[c].x;
                    searchPoint.y = cloud0->points[c].y;
                    searchPoint.z = cloud0->points[c].z;

                    if ( kdtree.radiusSearch (searchPoint, point_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) {
                        //printf("%d  666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666\n",pointIdxRadiusSearch.size());

                        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i) {

                            if(cloud1->points.size()>0) {
                                marked[pointIdxRadiusSearch[i]]=true;                     
                            }
                        }
                    }         
                }
                for(uint q=0; q< cloud1->points.size(); q++){
                    if(!marked[q]){
                        cloud_const->push_back(cloud1->points.at(q));
                    }
                }
                printf("constSize = %d ...................................................................................................", cloud_const->points.size());
                delete[] marked;
            }

            *frame2 = *frame + *cloud_const;

            for (size_t i = 0; i < (cloud_const->points.size()); i++){
                float distance = (sqrt((cloud_const->points[i].x - x1)*(cloud_const->points[i].x - x1) + 
                                       (cloud_const->points[i].y - y1)*(cloud_const->points[i].y - y1) + 
                                       (cloud_const->points[i].z - z1)*(cloud_const->points[i].z - z1)));

                if (distance < rsize) {
                    cloud_const->points[i].x = bad_point;
                    cloud_const->points[i].y = bad_point;
                    cloud_const->points[i].z = bad_point;
                }
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_const_filtered (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::removeNaNFromPointCloud(*cloud_const, *indices);
            pcl::ExtractIndices<pcl::PointXYZ> removeBotPts;
            removeBotPts.setInputCloud(cloud_const);
            removeBotPts.setIndices(indices);
            removeBotPts.setNegative(false);
            removeBotPts.filter(*cloud_const_filtered);
            *cloud_full += *cloud_const_filtered;
            *cloud0 = *cloud1;

            sensor_msgs::PointCloud2 cloud1Out;
            pcl::toROSMsg(*frame2, cloud1Out);
            pub2.publish (cloud1Out);
      
            cloud_const->clear();
            frame2->clear();  
        } 
    }

              

    //*cloud_filtered = *cloud;
    
    /*pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (30);
    sor.setStddevMulThresh (thres);
    sor.filter (*cloud_filtered);*/



    sensor_msgs::PointCloud2 output;
    

    pcl::toROSMsg(*cloud_full, output);
  

    pub1.publish (output);
  

}

void coord_cb (const std_msgs::Float32MultiArray& coord_msg) {
    x = coord_msg.data[0];
    y = coord_msg.data[1];
    z = coord_msg.data[2];
    printf("x=%f y=%f z=%f xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n",x,y,z);

}

int
main (int argc, char** argv) {
  // Initialize ROS
    ros::init (argc, argv, "mapFilter");
    ros::NodeHandle nh;

    nh.getParam("robot_size" , rsize);
    printf("size = %f \n",rsize);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub1 = nh.subscribe ("laser_cloud_surround", 1000, cloud_cb);
    ros::Subscriber sub2 = nh.subscribe ("coord", 1000, coord_cb);


    // Create a ROS publisher for the output point cloud
    pub1 = nh.advertise<sensor_msgs::PointCloud2> ("laser_cloud_map_filtered", 1000);
    pub2 = nh.advertise<sensor_msgs::PointCloud2> ("cloud1", 1000);

    // Spin
    ros::spin ();
}