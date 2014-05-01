#include <string.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl_ros/point_cloud.h"
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "CorrespGrouping.hpp"
#include "Main.hpp"
int main (int argc, char **argv){
	ros::init(argc, argv, "Stalker");
	ros::NodeHandle my_node;
	
	//Timer, Subscriber, Publisher description
	ros::Timer _imlost;
	ros::Subscriber pointcloud_sub;
	ros::Subscriber pc_filtered_sub;
	ros::Subscriber tracker2d_sub;
	ros::Publisher pose_pub;
	ros::Publisher newBB_pub;
	
	Main<pcl::PointXYZRGBA, pcl::SHOT352> main;
	pointcloud_sub = my_node.subscribe<sensor_msgs::PointCloud2> ("camera/depth/points_xyzrgb", 1, &Main<pcl::PointXYZRGBA, pcl::SHOT352>::doWork, &main);


	while(ros::ok()){
		ros::spinOnce();
	}

}
