#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include "Preprocessing.hpp"

BOOST_AUTO_TEST_CASE(trying)
{
	Preprocessing<pcl::PointXYZRGBA> p;
	
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	
	// Fill in the cloud data
	cloud->width  = 5;
	cloud->height = 5;
	cloud->points.resize (cloud->width * cloud->height);
	int y=0;
	int yy=0;
	int yyy=0;
	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
	cloud->points[i].x = y;
	cloud->points[i].y = std::numeric_limits<double>::quiet_NaN();
	cloud->points[i].z = yyy;
	y++;
	yyy++;
		if(y==5){
			y=0;
			yy++;
		}
	}
	
	
	BOOST_CHECK_EQUAL(p.gotnanTEST(cloud),1);
	BOOST_CHECK_EQUAL(p.gotinfTEST(cloud),0);

	
	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
	cloud->points[i].x = y;
	cloud->points[i].y = std::numeric_limits<double>::infinity();
	cloud->points[i].z = yyy;
	y++;
	yyy++;
		if(y==5){
			y=0;
			yy++;
		}
	}
	
	BOOST_CHECK_EQUAL(p.gotnanTEST(cloud),0);
	BOOST_CHECK_EQUAL(p.gotinfTEST(cloud),1);

	
		for (size_t i = 0; i < cloud->points.size (); ++i)
	{
	cloud->points[i].x = y;
	cloud->points[i].y = yy;
	cloud->points[i].z = yyy;
	y++;
	yyy++;
		if(y==5){
			y=0;
			yy++;
		}
	}

	BOOST_CHECK_EQUAL(p.gotnanTEST(cloud),0);
	BOOST_CHECK_EQUAL(p.gotinfTEST(cloud),0);

	
}