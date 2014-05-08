#include <string.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl_ros/point_cloud.h"
#include "pcl/io/ply_io.h"

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "CorrespGrouping.hpp"
#include "Main.hpp"
int main (int argc, char **argv){
	ros::init(argc, argv, "Stalker");
	ros::NodeHandle my_node;
	ros::NodeHandle priv_node("~");

	//Timer, Subscriber, Publisher description
	ros::Timer _imlost;
	ros::Subscriber pointcloud_sub;
	ros::Subscriber model_sub;
	ros::Subscriber pc_filtered_sub;
	ros::Subscriber tracker2d_sub;
	ros::Publisher pose_pub;
	ros::Publisher newBB_pub;
	
	std::string model="/home/ros/hydro_ws/catkin_ws/src/Tobot/stalker/src/Test/milk.pcd";
	Main<pcl::PointXYZRGBA, pcl::SHOT352> main;
	
	/*******PARAMETERS****************/
	
	std::string path2model;
	priv_node.param<std::string>("/model", path2model, "none");
		
	/*****************************************/
	
	/***********MODEL****************/
	if(path2model.compare("none")==0){
		model_sub = my_node.subscribe<sensor_msgs::PointCloud2> ("model", 1, &Main<pcl::PointXYZRGBA, pcl::SHOT352>::loadModel, &main);
	}
	else{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model_pc (new pcl::PointCloud<pcl::PointXYZRGBA>);
		main.loadModel(model_pc);
	}
	
	/***********CAMERA IMAGE*********/
	pointcloud_sub = my_node.subscribe<sensor_msgs::PointCloud2> ("camera/depth/points_xyzrgb", 1, &Main<pcl::PointXYZRGBA, pcl::SHOT352>::doWork, &main);


	while(ros::ok()){
		ros::spinOnce();
	}

}
