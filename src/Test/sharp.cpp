#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <string.h>
#include <iostream>
#include <time.h>
#include "sensor_msgs/PointCloud2.h"

#include "pcl_ros/point_cloud.h"

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/correspondence.h>
#include "Postprocessing.hpp"


BOOST_AUTO_TEST_CASE(trying)
{
	
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudin (new pcl::PointCloud<pcl::PointXYZRGBA>);
	//pcl::io::loadPCDFile ("/home/malcolm/ros_ws/hydro_ws/catkin_ws/src/Stalker/src/Test/milk.pcd", *object);
	pcl::io::loadPCDFile ("/mnt/Data/Mad Maker/PCL/Blender Models/starbucks_coord00000.pcd", *cloudin);
	
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object2 (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::io::loadPCDFile ("/mnt/Data/Mad Maker/PCL/Blender Models/starbucks_coord00000.pcd", *object2);
	
	if (cloudin->isOrganized()) {
		  for (int h=0; h<cloudin->height; h++) {
		      for (int w=0; w<cloudin->width; w++) {
		          cloudin->at(w,h).x = cloudin->at(w, h).x/2;
			  cloudin->at(w,h).y = cloudin->at(w, h).y/2;
			  cloudin->at(w,h).z = cloudin->at(w, h).z/2;
		      }
		  }
	}
	else{
		std::cerr << "Cloud not organized, can't apply the function : " <<cloudin->width<<" "<<cloudin->height<< std::endl;
		for (int h=0; h<cloudin->width; h++) {
		          cloudin->points[h].x = cloudin->points[h].x/2;
			  cloudin->points[h].y = cloudin->points[h].y/2;
			  cloudin->points[h].z = cloudin->points[h].z/2;
		}
	}
	
	stalker::voirPCL<pcl::PointXYZRGBA>(cloudin, object2);
}