#include <string.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl_ros/point_cloud.h"
#include "pcl/io/ply_io.h"

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Preprocessing.hpp"

#define PointType pcl::PointXYZRGBA

void mainCall(const sensor_msgs::PointCloud2ConstPtr& cloudy){
	
	pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>() );
	pcl::fromROSMsg(*cloudy, *model);
	pcl::transformPointCloud (*model, *model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
	stalker::passThrough<PointType>(model, model, "z", 0.8, 2);
	int a = pcl::io::savePCDFileBinary("view_model.pcd", *model);
	sensor_msgs::PointCloud2Ptr pc2(new sensor_msgs::PointCloud2());
}

int main (int argc, char **argv){
	
	ros::init(argc, argv, "Create model");
	ros::NodeHandle my_node;
	
	ros::Subscriber pointcloud_sub;
	pointcloud_sub = my_node.subscribe<sensor_msgs::PointCloud2> ("model", 1, boost::bind(mainCall, _1) );
	
	
	while(ros::ok()){		
		ros::spinOnce();
	}
}