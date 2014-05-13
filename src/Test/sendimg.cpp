#include <string.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl_ros/point_cloud.h"
#include "pcl/io/ply_io.h"

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void makeModel(const sensor_msgs::PointCloud2ConstPtr& cloudy, ros::Publisher& model_pub, ros::Publisher& cloud_pub){
	std::cout<<"HEllow"<<std::endl;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::fromROSMsg(*cloudy, *cloud);
	
	std::cout << "TAILLE DE "<< cloud->width<<" "<<cloud->height<<std::endl;
	double x_go=(cloud->width)/4;
	double y_go=(cloud->height)/4;

	
	double x_stop=((cloud->width)/2);
	double y_stop=((cloud->height)/2);
	
	cloud_filtered->width  = x_stop;
	cloud_filtered->height = y_stop;
	std::cout<<"hello "<< cloud_filtered->width<< "  "<<cloud_filtered->height <<std::endl;
	cloud_filtered->points.resize (cloud_filtered->width * cloud_filtered->height);
	std::cout<<"hello cloud filtered done"<<std::endl;
	
	for (size_t y = y_go; y < y_go+y_stop; ++y){
		for (size_t x = x_go; x < x_go+x_stop; ++x){
			/*cloud_filtered->points[i].x = cloud->points[x+(cloud->width)*y].x;
			cloud_filtered->points[i].y = cloud->points[x+(cloud->width)*y].y;
			cloud_filtered->points[i].z = cloud->points[x+(cloud->width)*y].z;*/
			cloud_filtered->at(x-x_go,y-y_go)=cloud->at(x,y);	
			//std::cout<<"hello loop "<<x <<" "<<y<<" to go to "<<x_go+x_stop<< " "<<y_go+y_stop<< " from " << x_go<<" "<<y_go<<" because size is "<< cloud_filtered->width<< "  "<<cloud_filtered->height<< std::endl;
		}
	}
	
	sensor_msgs::PointCloud2 pc2;
	pcl::toROSMsg(*cloud_filtered, pc2);
	
	pc2.header.stamp=ros::Time::now();
	pc2.header.frame_id=cloudy->header.frame_id;
	model_pub.publish(pc2);
	cloud_pub.publish(cloudy);
	
}



int main (int argc, char **argv){
	ros::init(argc, argv, "TestImg");
	ros::NodeHandle my_node;

	//Timer, Subscriber, Publisher description
	ros::Timer _imlost;
	ros::Subscriber pointcloud_sub;
	ros::Publisher model_pub=my_node.advertise<sensor_msgs::PointCloud2>("/model", 1000);
	ros::Publisher cloud_pub=my_node.advertise<sensor_msgs::PointCloud2>("/point_cloud", 1000);
	
		
	/*****************************************/


	/***********CAMERA IMAGE*********/
	pointcloud_sub = my_node.subscribe<sensor_msgs::PointCloud2> ("camera/depth/points_xyzrgb", 1, boost::bind(makeModel, _1, model_pub, cloud_pub) );


	while(ros::ok()){
		ros::spinOnce();
	}

}