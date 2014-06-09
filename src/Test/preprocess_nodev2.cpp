#include <string.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl_ros/point_cloud.h"
#include "pcl/io/ply_io.h"
#include "Preprocessing.hpp"
#include <Postprocessing.hpp>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


 #include <pcl/visualization/cloud_viewer.h>
 //...
 

int i=0;
double ds=0.03;

void testin(const sensor_msgs::PointCloud2ConstPtr& cloudy){
	std::cout<<"HEllow"<<std::endl;
	
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
	cloud->is_dense=false;
	cloud_filtered->is_dense=false;
	pcl::fromROSMsg(*cloudy, *cloud);

	//Xtion max and min range :  0.8 m to 3.5 m
	/*prep.removeNan(*cloud, *cloud);
	prep.removeNanNormals(*cloud, *cloud);
	prep.passThrough(cloud, cloud_filtered, "z", 1, 2 );*/
	
	stalker::removeNan<pcl::PointXYZRGBA>(*cloud, *cloud);
	
	
	
	

}



int main (int argc, char **argv){
	ros::init(argc, argv, "TestImg");
	ros::NodeHandle my_node;

	//Timer, Subscriber, Publisher description
	ros::Subscriber pointcloud_sub;
		
	/*****************************************/


	/***********CAMERA IMAGE*********/
	pointcloud_sub = my_node.subscribe<sensor_msgs::PointCloud2> ("camera/depth/points_xyzrgb", 1, testin);


	while(ros::ok()){
		ros::spinOnce();
	}

}