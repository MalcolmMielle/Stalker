#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include "Preprocessing.hpp"

BOOST_AUTO_TEST_CASE(trying)
{
	
	Preprocessing<pcl::PointXYZ> p;
	Preprocessing<pcl::PointXYZ> p2;
	//EXAMPLE
	typedef pcl::PointCloud<pcl::PointXYZ> CloudType;
	CloudType::Ptr cloudt (new CloudType);
	cloudt->is_dense = false;
	CloudType::Ptr output_cloud (new CloudType);

	CloudType::PointType p_nan;
	p_nan.x = std::numeric_limits<float>::quiet_NaN();
	p_nan.y = std::numeric_limits<float>::quiet_NaN();
	p_nan.z = std::numeric_limits<float>::quiet_NaN();
	cloudt->push_back(p_nan);

	CloudType::PointType p_valid;
	p_valid.x = 1.0f;
	cloudt->push_back(p_valid);
	
	BOOST_CHECK_EQUAL(p2.gotnanTEST(cloudt),1);

	std::cout << "size: " << cloudt->points.size () << std::endl;

	std::vector<int> indicest;
	pcl::removeNaNFromPointCloud(*cloudt, *cloudt, indicest);
	std::cout << "size: " << cloudt->points.size () << std::endl;
	
	BOOST_CHECK_EQUAL(p2.gotnanTEST(cloudt),0);
	
	std::cout<<std::endl<<std::endl<<"END OF TEST."<<std::endl;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
	cloud->is_dense = false;
	cloud_out->is_dense = false;
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
	
	p.removeNan(cloud);

	BOOST_CHECK_EQUAL(p.gotnanTEST(cloud),0);
	BOOST_CHECK_EQUAL(p.gotinfTEST(cloud),0);
	
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud,*cloud_out, indices);
	
	for(int i=0;i<indices.size();++i){
		std::cout<< indices[i]<<" ";
	}
	std::cout<<std::endl;
	
	
	
	BOOST_CHECK_EQUAL(p.gotnanTEST(cloud_out),0);
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
	//BOOST_CHECK_EQUAL(p.gotinfTEST(cloud),1);

	
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