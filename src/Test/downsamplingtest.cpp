#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include "Preprocessing.hpp"

BOOST_AUTO_TEST_CASE(trying)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
	cloud2->is_dense = false;

	cloud2->width  = 10;
	cloud2->height = 10;
	
	cloud2->points.resize (cloud2->width * cloud2->height);
	for (size_t i = 0; i < cloud2->width; ++i){
		for(size_t j=0; j<cloud2->height; ++j){
			cloud2->at(i,j).x = i;
			cloud2->at(i,j).y = j;
			cloud2->at(i,j).z = 1;
		}
	}
	
	double reso=stalker::computeCloudResolution<pcl::PointXYZ>(cloud2);
	std::cout<<reso<<std::endl;
	
	cloud2->at(4,4).x=10;
	cloud2->at(4,4).y=10;
	
	reso=stalker::computeCloudResolution<pcl::PointXYZ>(cloud2);
	std::cout<<reso<<std::endl;
}