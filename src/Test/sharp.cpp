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


BOOST_AUTO_TEST_CASE(trying)
{

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p(new pcl::PointCloud<pcl::PointXYZRGBA>() );
	
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p2(new pcl::PointCloud<pcl::PointXYZRGBA>() );
	

	

}