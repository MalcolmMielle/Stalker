#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include "Shape3DLocal.hpp"
#include <boost/test/unit_test.hpp>
 #include <pcl/visualization/cloud_viewer.h>
pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
BOOST_AUTO_TEST_CASE(trying)
{
	ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT352> st("test");
	ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT352> st2("test2");
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::io::loadPCDFile ("/home/malcolm/ros_ws/hydro_ws/catkin_ws/src/Stalker/src/Test/milk.pcd", *cloud);	
	
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::io::loadPCDFile ("/home/malcolm/ros_ws/hydro_ws/catkin_ws/src/Stalker/src/Test/milk_cartoon_all_small_clorox.pcd", *cloud2);
	
	st.update(cloud);
	st.loadMesh("/home/malcolm/ros_ws/hydro_ws/catkin_ws/src/Stalker/src/Test/milk.pcd");
	st.compute();
	std::cout<<"TESSSSST"<<std::endl;
	st.update(cloud);
	std::cout<<"TESSSSST2"<<std::endl;
	st.update(cloud);
	std::cout<<"END"<<std::endl;
	st.setRadius(3);
	st.setSamplingSize(5);
	BOOST_CHECK_EQUAL(st.getRadius(), 3);
	BOOST_CHECK_EQUAL(st.getSamplingSize(),5);
	
	st2.update(cloud2);
	
	while(!viewer.wasStopped()){
		viewer.showCloud (st.getCloud());
	}
	

}