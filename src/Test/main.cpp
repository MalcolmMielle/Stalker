#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include "MainGraphic.hpp"
#include "Shape3DLocal.hpp"
#include <boost/test/unit_test.hpp>
#include "CorrespGrouping.hpp"
#include "sensor_msgs/PointCloud2.h"

BOOST_AUTO_TEST_CASE(trying)
{
	
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>());
	
	pcl::io::loadPCDFile ("/home/ros/groovy_ws/catkin_ws/src/Tobot/stalker/src/Test/milk.pcd", *cloud);
	pcl::io::loadPCDFile ("/home/ros/groovy_ws/catkin_ws/src/Tobot/stalker/src/Test/milk.pcd", *cloud2);

	CorrespGrouping<pcl::PointXYZRGBA>* p = new CorrespGrouping<pcl::PointXYZRGBA>(new ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT352>("object"), new ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT352>("scene"));
	
	Main<pcl::PointXYZRGBA, pcl::SHOT352> mg(cloud, cloud2);//Mem lek
	
	mg.setScene(cloud);
	mg.setObject(cloud);
	
	mg.doWork();
	
	Main<pcl::PointXYZRGBA, pcl::SHOT352> mg2(p);
	mg2.setScene(cloud);
	mg2.setObject(cloud);
	mg2.doWork();
	
	Main<pcl::PointXYZRGBA, pcl::SHOT352> mg3(cloud, cloud, new CorrespGrouping<pcl::PointXYZRGBA>(new ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT352>("object"), new ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT352>("scene")));
	mg3.doWork();
	
	///////////////////////////////////////////////
	//MEMORY LEAKS SOMEWHERE
	Main<pcl::PointXYZRGBA, pcl::SHOT352> mainly; //Mem leak
	mainly.setMaxObject(5);
	mainly.setMaxScene(2);
	mainly.addObject(cloud);
	mainly.addObject(cloud);
	mainly.addObject(cloud);
	mainly.addScene(cloud);
	mainly.addScene(cloud);
	mainly.addScene(cloud);
	mainly.addScene(cloud);
	BOOST_CHECK_EQUAL(mainly.getAllObjects().size(),3);
	mainly.addObject(cloud);// <- Doesn't work because of the name...
	BOOST_CHECK_EQUAL(mainly.getPipeline()->getAllObjects().size(),4);
	BOOST_CHECK_EQUAL(mainly.getPipeline()->getAllScenes().size(),4);
	BOOST_CHECK_EQUAL(mainly.getAllScenes().size(),2);
	
	std::cout<<"Doing Work"<<std::endl << std::endl;
	sensor_msgs::PointCloud2Ptr smp(new sensor_msgs::PointCloud2);
	pcl::toROSMsg(*cloud2, *smp);
	
	mainly.setObject(cloud);
	
	mainly.doWork(smp); //UBUG
	mainly.doWork(smp);
	mainly.removeObject(cloud);
	BOOST_CHECK_EQUAL(mainly.getAllScenes().size(),2); //Size 2
	BOOST_CHECK_EQUAL(mainly.getPipeline()->getAllScenes().size(),6); //Size 2
	
	BOOST_CHECK_EQUAL(mg.getAllScenes().size(),0); //Size 0
	BOOST_CHECK_EQUAL(mg.getPipeline()->getAllScenes().size(),0); //Size 0
	
	BOOST_CHECK_EQUAL(mg2.getAllScenes().size(),0); //size 0
	BOOST_CHECK_EQUAL(mg2.getPipeline()->getAllScenes().size(),0); //size 0
	
	BOOST_CHECK_EQUAL(mg3.getAllScenes().size(),0);
	BOOST_CHECK_EQUAL(mg3.getPipeline()->getAllScenes().size(),0);
	
	
	
	/////////////////////////////
	CorrespGrouping<pcl::PointXYZRGBA>* cp=new CorrespGrouping<pcl::PointXYZRGBA>(new ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT352>("1"), new ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT352>("2"));
 	Main<pcl::PointXYZRGBA, pcl::SHOT352> meanie(cp);
};