#include <string.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl_ros/point_cloud.h"
#include "pcl/io/ply_io.h"
#include "Preprocessing.hpp"
#include <Preprocessing.hpp>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


 #include <pcl/visualization/cloud_viewer.h>
 //...
 
pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
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
	//doPreprocessing(*cloud, *cloud_filtered);
	
	std::cout<<"How much downsample for the size "<<cloud->width*cloud->height <<" cloud resolution = "<< stalker::computeCloudResolution<pcl::PointXYZRGBA>(cloud) << std::endl;
	
	stalker::passThrough<pcl::PointXYZRGBA>(cloud, cloud, "z", 0.8, 3.5);
	std::cout<<"Doing down sample"<<std::endl;
	
	std::cout<<"How much downsample for the size "<<cloud->width<<" "<<cloud->height <<" cloud resolution = "<< stalker::computeCloudResolution<pcl::PointXYZRGBA>(cloud) << std::endl;
	
	double b=stalker::computeCloudResolution<pcl::PointXYZRGBA>(cloud);
	double size=cloud->width*cloud->height;
	double radius=((0.7*8000)+(0.3*b))/size;
	
	std::cout << "radius "<<radius<<std::endl;
	
	stalker::downSample<pcl::PointXYZRGBA>(cloud, cloud_filtered, radius);
	std::cout<<"Doing the removal"<<std::endl;
	
	//stalker::statisticalOutilerRemoval<pcl::PointXYZRGBA>(cloud_filtered, cloud_filtered, 50, 1.0);
	
	std::cout<<"Quoi voir ?"<<std::endl;
	scanf("%d",&i);
	if(i==0){
		viewer.showCloud (cloud_filtered);
	}
	else{
		viewer.showCloud(cloud);
	}
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