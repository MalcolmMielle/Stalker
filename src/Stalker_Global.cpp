#include <string.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"

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
#include "Shape3DGlobal.hpp"

#include "stalker/square.h"
#include "stalker/getobject.h"


#define Descriptor pcl::Histogram<153>
#define PointType pcl::PointXYZRGBA

/****** GLOBAL VARIABLES *****/

//Declare you're new Pipeline there
CorrespGrouping<PointType, Descriptor >* cp= new CorrespGrouping<PointType, Descriptor >(new ShapeLocal<PointType, Descriptor >("object"), new ShapeLocal<PointType, Descriptor >("scene", 0.03));

std::string frame;

/*******Function********/



/**************µCALLBACKS************************/

bool getPose(stalker::getobject::Request  &req, stalker::getobject::Response &res){
	ROS_INFO("Request");
	
	if(req.ask==true && cp->foundObject()){
		geometry_msgs::PoseStamped pose_stamped;
				
		pose_stamped.header.frame_id=frame;
		
		pose_stamped.pose.position.x=0;
		pose_stamped.pose.position.y=0;
		pose_stamped.pose.position.z=0;
		
		pose_stamped.pose.orientation.x=0;
		pose_stamped.pose.orientation.y=0;
		pose_stamped.pose.orientation.z=0;
		pose_stamped.pose.orientation.w=1;
		
		//TODO What if multiples objects ?
		stalker::calculatePose(cp->getRoto()[0], pose_stamped.pose ,pose_stamped.pose );
		
		pose_stamped.header.stamp=ros::Time::now();
		
		res.gotObject=true;
		res.pose=pose_stamped;
		res.who="Stalker";		
	}	
	
	/*if(!strcmp(req.order.c_str(),"face")){
		modelPath=req.model;
		tld->release();
		tld->readFromFile(modelPath);
	}
	else if(!strcmp(req.order.c_str(),"stoplook")){
		ROS_INFO("Stop la reconnaissance");
		tld->release();
	}	
	res.answer=req.order;
	return true;*/
	
}



void saveCloud(const sensor_msgs::PointCloud2ConstPtr& cloudy, pcl::PointCloud<PointType>::Ptr cloud){
	 pcl::fromROSMsg(*cloudy, *cloud); 
	 frame=cloudy->header.frame_id;
}




void bombCallBack(const ros::TimerEvent&, ros::Time& timestamp, ros::NodeHandle& my_node, Main<PointType, Descriptor >* main, ros::Publisher& bb_pub, ros::Publisher& pose_pub, pcl::PointCloud<PointType>::Ptr cloud){
	if( ros::Time::now()-timestamp>ros::Duration(10)){
		std::cout<<"************************************* Opent TLD did NOT found a Model********************************"<<std::endl;
		
		/*Preprocessing here...*/
		
		stalker::passThrough<PointType>(cloud, cloud, "z", 0.8, 3.5);
		stalker::statisticalOutilerRemoval<PointType>(cloud, cloud, 50, 1.0);
		
		main->doWork(cloud);
		
		if(main->foundObject()){
		
			/*** BOUNDING BOX****/
			stalker::square square=cp->getBoundingBox();
			bb_pub.publish<stalker::square>(square);
			
			/****POSE****/
			//TODO For now it assumed that the model is centered at the begining. Need the either automatise the process or to calculate the first pose.
			geometry_msgs::PoseStamped pose_stamped;
			
			pose_stamped.header.frame_id=frame;
			
			pose_stamped.pose.position.x=0;
			pose_stamped.pose.position.y=0;
			pose_stamped.pose.position.z=0;
			
			pose_stamped.pose.orientation.x=0;
			pose_stamped.pose.orientation.y=0;
			pose_stamped.pose.orientation.z=0;
			pose_stamped.pose.orientation.w=1;
			
			//TODO What if multiples objects ?
			stalker::calculatePose(cp->getRoto()[0], pose_stamped.pose ,pose_stamped.pose );
			
			pose_stamped.header.stamp=ros::Time::now();
			
			pose_pub.publish<geometry_msgs::PoseStamped>(pose_stamped);
		}
		
	}
	
	
}

void mainCall(const sensor_msgs::PointCloud2ConstPtr& cloudy, ros::Time& timestamp, Main<PointType, Descriptor >* main, ros::Publisher& pose_pub){
	
	std::cout<<"************************************* Opent TLD got a Model********************************"<<std::endl;

	if(main->gotModel()){
		pcl::PointCloud<PointType>::Ptr _scene(new pcl::PointCloud<PointType>() );
		_scene->is_dense=false;
		pcl::fromROSMsg(*cloudy, *_scene);
		//TODO
		stalker::resizeCloud<PointType>(main->getObject(), _scene);
		
		stalker::center<PointType>(main->getObject(), "x");
		stalker::center<PointType>(main->getObject(), "y");
		stalker::center<PointType>(main->getObject(), "z");
		
		stalker::voirPCL<PointType>(main->getObject(), _scene);
		
		sensor_msgs::PointCloud2Ptr pc2(new sensor_msgs::PointCloud2());
		pcl::toROSMsg(*_scene, *pc2);
		
		/*sensor_msgs::PointCloud2 pc2;
		pcl::toROSMsg(*cloud_filtered, pc2);*/
		
		main->doWork(pc2);
		
		if(main->foundObject()){
		//publish the new bouding box
			//pose_pub.publish<>();
		}
		
	}
	
	timestamp=ros::Time::now();
}


/*
 * 
 * 
 * MAINtemplate <typename T, typename DescriptorType>
 * 
 * 
 */

int main (int argc, char **argv){
	ros::init(argc, argv, "Stalker_global");
	ros::NodeHandle my_node;
	ros::NodeHandle priv_node("~");

	//Timer, Subscriber, Publisher description
	ros::Timer _imlost;
	ros::Subscriber pointcloud_sub;
	ros::Subscriber model_sub;
	ros::Subscriber pc_filtered_sub;
	ros::Subscriber tracker2d_sub;
	ros::Subscriber trackerfullcloud;
	
	
	ros::Publisher pose_pub;
	ros::Publisher newBB_pub;
	
	//Service
	ros::ServiceServer service = my_node.advertiseService("getObject", getPose);
	ROS_INFO("Ready to give the pose");
	
	ros::Timer bomb;
	ros::Time timeStamp=ros::Time::now();
	
	pcl::PointCloud<PointType>::Ptr cloudy_save;
	
	MainGraphic<PointType, Descriptor > main(cp);
	

	
	main.setPipeline(cp);
	/*******PARAMETERS****************/
	
	std::string path2model="/home/malcolm/ros_ws/hydro_ws/catkin_ws/devel/lib/stalker/view_model.pcd";

	std::string where2read="cloud_filtered";
	//priv_node.param<std::string>("/model", path2model, "none");
	
	//main.setResolution(true);
	/* SPIN IMAGE*/
	cp->setPostProcICPThresh(1e-7);
	cp->getObject()->setRadiusDescriptorsEffective(0.05);
	cp->getScene()->setRadiusDescriptorsEffective(0.05);
	cp->getObject()->setSamplingSizeEffective(0.01);
	cp->getScene()->setSamplingSizeEffective(0.01);
	
	/*SHOT*/	
	/*cp->setPostProcICPThresh(1e-7);
	//cp->getObject()->setRadiusDescriptorsEffective(0.03);
	//cp->getScene()->setRadiusDescriptorsEffective(0.05);
	cp->getObject()->setSamplingSizeEffective(0.005);
	cp->getScene()->setSamplingSizeEffective(0.005);*/
	
	cp->setAlwaysSeeBest();
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
	pointcloud_sub = my_node.subscribe<sensor_msgs::PointCloud2> (where2read, 1, boost::bind(mainCall, _1, timeStamp, &main, pose_pub) );

	bomb=my_node.createTimer(ros::Duration(10), boost::bind(bombCallBack, _1, timeStamp, my_node, &main, newBB_pub, pose_pub, cloudy_save));
	
	trackerfullcloud = my_node.subscribe<sensor_msgs::PointCloud2> ("camera/depth/points_xyzrgb", 1, boost::bind(saveCloud, _1,cloudy_save));
	

	while(ros::ok()){		
		
		
		ros::spinOnce();
	}
	
	delete(cp);

}
