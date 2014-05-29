#include <string.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl_ros/point_cloud.h"
#include "pcl/io/ply_io.h"

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "CorrespGrouping.hpp"
#include "MainGraphic.hpp"
#include "Main.hpp"
#include "Postprocessing.hpp"


#define Descriptor pcl::SHOT1344
#define PointType pcl::PointXYZRGBA




void bombCallBack(const ros::TimerEvent&, ros::Time& timestamp, ros::NodeHandle& my_node, Main<PointType, Descriptor >* main){
	if( ros::Time::now()-timestamp>ros::Duration(10)){
		std::cout<<"************************************* Opent TLD did NOT found a Model********************************"<<std::endl;
		ros::Subscriber trackertemp;
		
		//TODO FIX THAT
			//stalker::passThrough<T>(_scene, _scene, "z", 0.8, 3.5);
	//stalker::statisticalOutilerRemoval<T>(_scene, _scene, 50, 1.0);
		trackertemp = my_node.subscribe<sensor_msgs::PointCloud2> ("camera/depth/points_xyzrgb", 1, &Main<PointType, Descriptor >::doWork, main);
		ros::spinOnce();
	}
	
	
}

void mainCall(const sensor_msgs::PointCloud2ConstPtr& cloudy, ros::Time& timestamp, Main<PointType, Descriptor >* main){
	
	std::cout<<"************************************* Opent TLD got a Model********************************"<<std::endl;

	if(main->gotModel()){
		pcl::PointCloud<PointType>::Ptr _scene(new pcl::PointCloud<PointType>() );
		pcl::PointCloud<PointType>::Ptr _scene_resize(new pcl::PointCloud<PointType>() );
		_scene->is_dense=false;
		_scene_resize->is_dense=false;
		pcl::fromROSMsg(*cloudy, *_scene);
		//TODO
		double factor=1;
		stalker::resize<PointType>(_scene, _scene_resize, factor);
		
		sensor_msgs::PointCloud2Ptr pc2(new sensor_msgs::PointCloud2());
		pcl::toROSMsg(*_scene_resize, *pc2);
		
		/*sensor_msgs::PointCloud2 pc2;
		pcl::toROSMsg(*cloud_filtered, pc2);*/
		
		main->doWork(pc2);
	}
	else{
		pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>() );
		pcl::fromROSMsg(*cloudy, *model);
		pcl::transformPointCloud (*model, *model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
		stalker::passThrough<PointType>(model, model, "z", 0.8, 2);
		int a = pcl::io::savePCDFileBinary("view_model.pcd", *model);
		sensor_msgs::PointCloud2Ptr pc2(new sensor_msgs::PointCloud2());
		pcl::toROSMsg(*model, *pc2);
		main->loadModel(pc2);
	}
	
	timestamp=ros::Time::now();
}


int main (int argc, char **argv){
	ros::init(argc, argv, "Stalker");
	ros::NodeHandle my_node;
	ros::NodeHandle priv_node("~");

	//Timer, Subscriber, Publisher description
	ros::Timer _imlost;
	ros::Subscriber pointcloud_sub;
	ros::Subscriber model_sub;
	ros::Subscriber pc_filtered_sub;
	ros::Subscriber tracker2d_sub;
	ros::Publisher pose_pub;
	ros::Publisher newBB_pub;
	ros::Timer bomb;
	ros::Time timeStamp=ros::Time::now();
	
	MainGraphic<PointType, Descriptor > main;
	
	CorrespGrouping<PointType, Descriptor >* cp= new CorrespGrouping<PointType, Descriptor >(new ShapeLocal<PointType, Descriptor >("object"), new ShapeLocal<PointType, Descriptor >("scene", 0.03));
	
	main.setPipeline(cp);
	/*******PARAMETERS****************/
	
	std::string path2model="/home/malcolm/ros_ws/hydro_ws/catkin_ws/devel/lib/stalker/view_model.pcd";

	std::string where2read="cloud_filtered";
	//priv_node.param<std::string>("/model", path2model, "none");
	
	//main.setResolution(true);
	/* SPIN IMAGE
	cp->setPostProcICPThresh(1e-7);
	cp->getObject()->setRadiusDescriptorsEffective(0.05);
	cp->getScene()->setRadiusDescriptorsEffective(0.05);
	cp->getObject()->setSamplingSizeEffective(0.01);
	cp->getScene()->setSamplingSizeEffective(0.01);*/
		
	cp->setPostProcICPThresh(1e-7);
	//cp->getObject()->setRadiusDescriptorsEffective(0.03);
	//cp->getScene()->setRadiusDescriptorsEffective(0.05);
	cp->getObject()->setSamplingSizeEffective(0.001);
	cp->getScene()->setSamplingSizeEffective(0.001);
	
	cp->useGeometricConsistency();
	
	
	/*****************************************/
	
	/***********MODEL****************/
	if(path2model.compare("none")==0){
		model_sub = my_node.subscribe<sensor_msgs::PointCloud2> ("model", 1, &MainGraphic<PointType, Descriptor >::loadModel, &main);
	}
	else{
		std::cout << "Loading MODEL"<<std::endl;
		pcl::PointCloud<PointType>::Ptr model_pc (new pcl::PointCloud<PointType>);
		pcl::io::loadPCDFile (path2model, *model_pc);
		main.loadModel(model_pc);
	}
	
	/***********CAMERA IMAGE*********/
	pointcloud_sub = my_node.subscribe<sensor_msgs::PointCloud2> (where2read, 1, boost::bind(mainCall, _1, timeStamp, &main) );

	bomb=my_node.createTimer(ros::Duration(10), boost::bind(bombCallBack, _1, timeStamp, my_node, &main));
	

	while(ros::ok()){		
		
		
		ros::spinOnce();
	}

}
