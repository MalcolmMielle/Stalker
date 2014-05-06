#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
//#include "MainGraphic.hpp"
#include "Shape3DLocal.hpp"
#include <boost/test/unit_test.hpp>
#include "CorrespGrouping.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

#include "Gui1.hpp"
#include "MainGraphic.hpp"

typedef pcl::PointXYZRGBA PointType;
bool show_keypoints_ (true);
bool show_correspondences_ (true);
bool use_cloud_resolution_ (true);
bool use_hough_ (true);
float model_ss_ (0.01f);
float scene_ss_ (0.03f);
float rf_rad_ (0.015f);
float descr_rad_ (0.02f);
float cg_size_ (0.01f);
float cg_thresh_ (5.0f);


BOOST_AUTO_TEST_CASE(trying)
{
	
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::io::loadPCDFile ("/home/ros/hydro_ws/catkin_ws/src/Tobot/stalker/src/Test/milk.pcd", *object);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::io::loadPCDFile ("/home/ros/hydro_ws/catkin_ws/src/Tobot/stalker/src/Test/milk_cartoon_all_small_clorox.pcd", *cloud2);
	
	CorrespGrouping<pcl::PointXYZRGBA> cg(new ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT352>("object"), new ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT352>("scene",0.03));
	
	//std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	
	//bool show_correspondences_=true;
	//bool show_keypoints_=true;
	/***FUNCTION***/
	/*std::cout<<"Set functions"<<std::endl;
	cg.setObject(object);
	cg.setScene(cloud2);
	
	cg.doPipeline();*/
	
	/***********Print*/
	std::cout << "PRIIIIINT"<<std::endl;
	/*gui1.printCorresp(cg);
	gui1.add(*(cg.getObject()));
	gui1.add(*(cg.getScene()));*/
	//gui1.printKeyPoints();
	
	//gui1.show();
	
	MainGraphic<PointType, pcl::SHOT352> mg;
	mg.setPipeline(&cg);
	mg.setObject(object);
	mg.setScene(cloud2);
	mg.doWork();
	
	mg.getPipeline()->affiche();
	
	//mg.getGui()->printPipeline(cg);
	
	while(1){
		mg.getGui()->show();
	}
	
	
}