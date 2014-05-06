#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
//#include "MainGraphic.hpp"
#include "Shape3DLocal.hpp"
#include <boost/test/unit_test.hpp>
#include "CorrespGrouping.hpp"

BOOST_AUTO_TEST_CASE(trying)
{
	ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT352> st("test");
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::io::loadPCDFile ("/home/ros/hydro_ws/catkin_ws/src/Tobot/stalker/src/Test/milk.pcd", *cloud);	
	
	
	CorrespGrouping<pcl::PointXYZRGBA> cg(new ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT352>("bob1"), new ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT352>("bob2"));
	cg.setMaxObject(2);
	cg.setMaxScene(4);
	//cg.addObject(new ShapeLocal<pcl::PointXYZRGBA>("bob2")); //Problem with this declaration oO
	//BOOST_CHECK_EQUAL(cg.getAllObjects().size(),1);
	
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>());
	
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	//pcl::io::loadPCDFile ("/home/ros/groovy_ws/catkin_ws/src/Tobot/stalker/src/Test/milk.pcd", *cloud);
	
	//pcl::io::loadPCDFile ("/home/ros/groovy_ws/catkin_ws/src/Tobot/stalker/src/Test/milk.pcd", *cloud2);
	std::cout<<"lest add"<<std::endl;
	cg.addObject(cloud);
	cg.addObject(cloud);
	cg.addObject(cloud);
	cg.addObject(cloud);
	cg.addObject(cloud);
	
	BOOST_CHECK_EQUAL(cg.getAllObjects().size(), 2);
	
	std::cout<<"lest add2"<<std::endl;
	cg.addScene(cloud);
	cg.addScene(cloud);
	cg.addScene(cloud);
	cg.addScene(cloud);
	cg.addScene(cloud);
	
	BOOST_CHECK_EQUAL(cg.getAllScenes().size(), 4);
	
	std::cout<<"First printinfo"<<std::endl;
	cg.printinfo();
	
	cg.removeObject(0);
	cg.removeObject(0);
	cg.removeObject(0);
	cg.removeObject(0);
	cg.removeObject(0);
	
	std::cout<<"Second printinfo"<<std::endl;
	cg.printinfo();
	
	cg.addObject(cloud);
	cg.addObject(cloud);
	cg.addObject(cloud);
	cg.addObject(cloud);
	cg.addObject(cloud);
	

	std::cout<<"Third printinfo"<<std::endl;
	cg.printinfo();
	cg.clearObjects();
	std::cout<<"fourth printinfo"<<std::endl;
	cg.printinfo();
	
	//BOOST_CHECK_EQUAL(cg.getAllObjects().size(),2);
	//cg.addObject(cloud2);
	
	std::cout<<"Ten End"<<std::endl;
}