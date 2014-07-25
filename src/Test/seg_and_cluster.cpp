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
#include "Preprocessing.hpp"

#include <SegmentAndClustering.hpp>
#include <Shape3DGlobal.hpp>

typedef pcl::PointXYZRGBA PointType;
typedef pcl::SHOT352 Descriptor;

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
	pcl::io::loadPCDFile ("/home/malcolm/ros_ws/hydro_ws/catkin_ws/src/Stalker/src/Test/milk.pcd", *object);
	//pcl::io::loadPCDFile ("/mnt/Data/Mad Maker/PCL/Blender Models/starbucks_coord00000.pcd", *object);
	
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>);
	//pcl::io::loadPCDFile ("/mnt/Data/Mad Maker/PCL/Blender Models/starbucks_coord200000.pcd", *cloud2);
	pcl::io::loadPCDFile ("/home/malcolm/ros_ws/hydro_ws/catkin_ws/src/Stalker/src/Test/milk_cartoon_all_small_clorox.pcd", *cloud2);
	
	//CorrespGrouping<pcl::PointXYZRGBA, Descriptor> cg(new ShapeLocal<pcl::PointXYZRGBA, Descriptor>("bob1"), new ShapeLocal<pcl::PointXYZRGBA, Descriptor>("bob2",0.015));
	
	SegmentAndClustering<pcl::PointXYZRGBA, Descriptor> cg(new ShapeGlobal<PointType, Descriptor>("bob1"), new ShapeGlobal<PointType, Descriptor>("bob2"));
	CorrespGrouping<pcl::PointXYZRGBA, Descriptor> tg(new ShapeLocal<pcl::PointXYZRGBA, Descriptor>("bob1"), new ShapeLocal<pcl::PointXYZRGBA, Descriptor>("bob2"));
	
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	
	bool show_correspondences_=true;
	bool show_keypoints_=true;
	
	//cg.setPostProcICPThresh(0.00007);
	cg.getObject()->setRadiusDescriptorsEffective(0.05);
	cg.getScene()->setRadiusDescriptorsEffective(0.05);
	cg.getObject()->setSamplingSizeEffective(0.01);
	cg.getScene()->setSamplingSizeEffective(0.01);
	
	/***FUNCTION***/


	cg.setObject(object);
	cg.setScene(cloud2);
	
	cg.doPipeline();
	cg.doPipeline();
	
	cg.setObject(cloud2);
	cg.setScene(object);
	
	cg.doPipeline();
	
	cg.setObject(object);
	cg.setScene(cloud2);
	
	cg.doPipeline();
	
	/*********/
	
	rototranslations=cg.getRoto();
	//For correspondance grouping 
	//std::vector<pcl::Correspondences> clustered_corrs=cg.getClust();
	
	//For Seg and clustering
	std::vector<typename pcl::PointCloud<PointType> > clustered_corrs;
	clustered_corrs=cg.getClusters();
	
	pcl::PointCloud<PointType>::Ptr model = cg.getObject()->getCloud();
	pcl::PointCloud<PointType>::Ptr model_keypoints=cg.getObject()->getKeypoints();
	
	pcl::PointCloud<PointType>::Ptr scene = cg.getScene()->getCloud();
	pcl::PointCloud<PointType>::Ptr scene_keypoints=cg.getScene()->getKeypoints();
	
	/*std::cout << "Model instances found: " << rototranslations.size () << std::endl;
	for (size_t i = 0; i < rototranslations.size (); ++i)
	{
		std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
		std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
		Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

		printf ("\n");
		printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
		printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
		printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
		printf ("\n");
		printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
	}*/
	pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
	viewer.addPointCloud (scene, "scene_cloud");
	
	viewer.addCoordinateSystem (1.0);

	pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());
	
	

	if (show_correspondences_ || show_keypoints_)
	{
	//  We are translating the model so that it doesn't end in the middle of the scene representation
		pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
		pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
		viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
	}

	/*if (show_keypoints_)
	{
		pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
		viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
		viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
	}*/

	for (size_t i = 0; i < rototranslations.size (); ++i)
	{
		pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
		pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

		std::stringstream ss_cloud;
		ss_cloud << "instance" << i;

		pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
		viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

		if (show_correspondences_)
		{
			
		
			
			
		/*	for (size_t j = 0; j < clustered_corrs[i].size (); ++j)
			{
				std::stringstream ss_line;
				ss_line << "correspondence_line" << i << "_" << j;
				//PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
				//PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

				//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
				viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
			}*/
		}
		
		//add a sphere to show the result of the translation
		std::stringstream ss_sphere;
		ss_sphere << "instance" << i;
		pcl::PointXYZ c;
		Eigen::Matrix4f t = rototranslations[i];
		c.x = t(0,3);
		c.y = t(1,3);
		c.z = t(2,3);
		viewer.addSphere(c, 0.1, ss_sphere.str());
	}
	
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudmot (new pcl::PointCloud<pcl::PointXYZRGBA>);
	
	/*for (size_t i = 0; i < clustered_corrs.size (); ++i){
				std::stringstream ss_cloud_v;
				ss_cloud_v << "cluster" << i;
				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudmot (new pcl::PointCloud<pcl::PointXYZRGBA>);
				pcl::copyPointCloud(clustered_corrs[i], *cloudmot);
				pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (cloudmot, 0, 255, 0);
				viewer.addPointCloud (cloudmot, rotated_model_color_handler, ss_cloud_v.str ());
			}*/
	
	
	/*std::cout <<"Drawing boudning box"<<std::endl;
	//Draw boudning box : 
	stalker::square sq;
	sq=cg.getBoundingBox();
	
	std::cout <<"Got it"<<std::endl;
	PointType p1;
	p1.x=sq.point.x;
	std::cout <<"setX"<<std::endl;
	p1.y=sq.point.y;
	p1.z=sq.point.z;
	
	PointType p2;
	p2.x=sq.point.x+sq.width;
	p2.y=sq.point.y;
	p2.z=sq.point.z;
	
	PointType p3;
	p3.x=sq.point.x+sq.width;
	p3.y=sq.point.y-sq.height;
	p3.z=sq.point.z;
	
	PointType p4;
	p4.x=sq.point.x;
	p4.y=sq.point.y-sq.height;
	p4.z=sq.point.z;
	std::cout <<"Putting name"<<std::endl;
	std::stringstream ss_line1;
	ss_line1 << "boudingbox1";
	std::stringstream ss_line2;
	ss_line2 << "boudingbox2";
	std::stringstream ss_line3;
	ss_line3 << "boudingbox3";
	std::stringstream ss_line4;
	ss_line4 << "boudingbox4";
	
	viewer.addLine<PointType, PointType> (p1, p2, 255, 255, 0, ss_line1.str ());
	viewer.addLine<PointType, PointType> (p2, p3, 255, 255, 0, ss_line2.str ());
	viewer.addLine<PointType, PointType> (p3, p4, 255, 255, 0, ss_line3.str ());
	viewer.addLine<PointType, PointType> (p4, p1, 255, 255, 0, ss_line4.str ());*/
	


	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
	}
	
}
	
	
	

  
