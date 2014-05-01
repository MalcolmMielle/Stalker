#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include "Gui.hpp"
#include <boost/test/unit_test.hpp>


BOOST_AUTO_TEST_CASE(trying)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::io::loadPCDFile ("/home/ros/catkin_ws/src/Tobot/stalker/src/Test/view.pcd", *cloud);
	//
	Gui<pcl::PointXYZRGBA> gui;
	gui.add(cloud, "well");
	while(gui.viewer->wasStopped()){
	//pcl::io::loadPCDFile ("milk.pcd", *cloud);
		gui.show();
	}

}
