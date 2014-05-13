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
#include "pcl/io/ply_io.h"
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
	
	/*CorrespGrouping<pcl::PointXYZRGBA, pcl::SHOT352>* cg = new CorrespGrouping<pcl::PointXYZRGBA, pcl::SHOT352>(new ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT352>("object"), new ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT352>("scene",0.03));
	
	MainGraphic<PointType, pcl::SHOT352> mg;
	mg.setPipeline(cg);
	mg.setObject(object);
	mg.setScene(cloud2);
	while(!mg.stopGui()){
		mg.getGui()->show();
	}
	mg.doWork();
	
	mg.getPipeline()->affiche();
	
	//mg.getGui()->printPipeline(cg);
	
	while(!mg.stopGui()){
		mg.getGui()->show();
	}
	
	std::cout<<"LETS CONTINUE"<<std::endl<<std::endl;
	CorrespGrouping<pcl::PointXYZRGBA, pcl::SHOT352>* cg2  = new CorrespGrouping<pcl::PointXYZRGBA, pcl::SHOT352>(new ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT352>("object"), new ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT352>("scene",0.03));
	
	MainGraphic<PointType, pcl::SHOT352> mg2(object, cloud2);
	
	mg2.setPipeline(cg2);
	//std::cout<<"LETS ADD OBJECT"<<std::endl<<std::endl;
	mg2.addObject(object);
	std::cout<<"LETS ADD SCENE"<<std::endl<<std::endl;
	mg2.addScene(cloud2);
	//std::cout<<"LETS DO WORK"<<std::endl<<std::endl;
	mg2.doWork();
	mg2.getPipeline()->affiche();
	while(!mg2.stopGui()){
		mg2.getGui()->show();
	}
	
	/*****LOAD PLY TRYING***/
	/******LOAD MODEL TO BE CHANGED***********/
	/*
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGBA>);
	//pcl::io::loadPLYFile ("/home/ros/hydro_ws/catkin_ws/src/Tobot/stalker/src/Test/mugg.ply", *model);	
	pcl::io::loadPCDFile ("/home/ros/hydro_ws/catkin_ws/src/Tobot/stalker/src/Test/cube.pcd", *model);
	
	Gui1<pcl::PointXYZRGBA, pcl::SHOT352> g1;
	g1.Gui<pcl::PointXYZRGBA, pcl::SHOT352>::addPCL(model, "model");
	
	while(!g1.wasStopped()){
		g1.show();
	}
	
	
	/*****TRYING ANOTHER DESCRIPTOR********/
	/*
	std::cout<<"LETS CONTINUE COLOR"<<std::endl<<std::endl;
	CorrespGrouping<pcl::PointXYZRGBA, pcl::SHOT1344>* cg3  = new CorrespGrouping<pcl::PointXYZRGBA, pcl::SHOT1344>(new ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT1344>("object"), new ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT1344>("scene",0.03));
	
	MainGraphic<PointType, pcl::SHOT1344> mg3(object, cloud2);
	
	mg3.setPipeline(cg3);
	//std::cout<<"LETS ADD OBJECT"<<std::endl<<std::endl;
	mg3.setObject(object);
	std::cout<<"LETS ADD SCENE"<<std::endl<<std::endl;
	mg3.setScene(cloud2);
	//std::cout<<"LETS DO WORK"<<std::endl<<std::endl;
	mg3.doWork();
	mg3.getPipeline()->affiche();
	while(!mg3.stopGui()){
		mg3.getGui()->show();
	}
	
		/*****TRYING ANOTHER DESCRIPTOR********/
		/*
	std::cout<<"LETS CONTINUE SPIN IMAGE"<<std::endl<<std::endl;
	CorrespGrouping<pcl::PointXYZRGBA,pcl::Histogram<153> >* cg4  = 
		new CorrespGrouping<pcl::PointXYZRGBA,pcl::Histogram<153> >(
			new ShapeLocal<pcl::PointXYZRGBA,pcl::Histogram<153> >("object"), 
			new ShapeLocal<pcl::PointXYZRGBA,pcl::Histogram<153> >("scene",0.03)
		);
	
	MainGraphic<PointType,pcl::Histogram<153> > mg4(object, cloud2);
	
	mg4.setPipeline(cg4);
	//std::cout<<"LETS ADD OBJECT"<<std::endl<<std::endl;
	mg4.setObject(object);
	std::cout<<"LETS ADD SCENE"<<std::endl<<std::endl;
	mg4.setScene(cloud2);
	//std::cout<<"LETS DO WORK"<<std::endl<<std::endl;
	mg4.doWork();
	mg4.getPipeline()->affiche();
	while(!mg4.stopGui()){
		mg4.getGui()->show();
	}*/
	
	/******Trying the real functions*****/
	MainGraphic<PointType,pcl::SHOT352 > mg5;
	
	CorrespGrouping<pcl::PointXYZRGBA, pcl::SHOT352>* cg5  = new CorrespGrouping<pcl::PointXYZRGBA, pcl::SHOT352>(new ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT352>("object"), new ShapeLocal<pcl::PointXYZRGBA, pcl::SHOT352>("scene",0.03));
	
	sensor_msgs::PointCloud2Ptr smp(new sensor_msgs::PointCloud2);
	pcl::toROSMsg(*cloud2, *smp);
	std::cout<<std::endl<<"LOAD MODEL"<<std::endl<<std::endl;
	mg5.loadModel(object);
	std::cout<<std::endl<<"DO WORKL"<<std::endl<<std::endl;
	//mg5.setMaxObject(5);
	
	mg5.doWork(smp); //UBUG
	mg5.getPipeline()->affiche();
	while(!mg5.stopGui()){
		mg5.getGui()->show();
	}
	
}