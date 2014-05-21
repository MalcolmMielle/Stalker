#include <string.h>
#include <iostream>
#include <time.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl_ros/point_cloud.h"
#include "pcl/io/ply_io.h"
#include "Preprocessing.hpp"
#include <Shape3DLocal.hpp>
#include <CorrespGrouping.hpp>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


 #include <pcl/visualization/cloud_viewer.h>
 //...
 
pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT1344> shape("scene", 0.03);

//CorrespGrouping<pcl::PointXYZRGBA, pcl::SHOT352> cp(new ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT352>("object"), new ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT352>("scene", 0.03));
struct timespec now;
double tt_tic=0;

double getTickCount(void) 
{
  if (clock_gettime(CLOCK_MONOTONIC, &now))
    return 0;
  return now.tv_sec * 1000.0 + now.tv_nsec / 1000000.0;
}
 
void tic(){
	if (clock_gettime(CLOCK_MONOTONIC, &now))
		tt_tic= -1;
	tt_tic= now.tv_sec * 1000.0 + now.tv_nsec / 1000000.0;
}
void toc(){
	if(tt_tic==-1)
		std::cout<<"Clock error"<<std::endl;
	double tt_toc = getTickCount() - tt_tic;
	printf ("shape calculated in : %4.3f ms\n", tt_toc);
}

void testin(const sensor_msgs::PointCloud2ConstPtr& cloudy){

	std::cout<<"HEllow"<<std::endl;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	cloud->is_dense=false;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p(new pcl::PointCloud<pcl::PointXYZRGBA>() );
	p->is_dense=false;
	
	pcl::fromROSMsg(*cloudy, *cloud);
	pcl::copyPointCloud(*cloud, *p);
	tic();
	shape.update(cloud);
	toc();
	//cp.setScene(p);
	
	
	viewer.showCloud (cloud);
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