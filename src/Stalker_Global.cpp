#include <string.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"

#include "pcl_ros/point_cloud.h"
#include "pcl/io/ply_io.h"

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "CorrespGrouping.hpp"
#include "MainGraphic.hpp"
#include "Main.hpp"
#include "Postprocessing.hpp"

#include <SegmentAndClustering.hpp>
#include <Shape3DGlobal.hpp>

#include <tf/transform_listener.h>

#include "stalker/square.h"
#include "stalker/getobject.h"
#include "stalker/sentobject.h"


#define Descriptor pcl::Histogram<153>
#define PointType pcl::PointXYZRGBA

/****** GLOBAL VARIABLES *****/

//Declare you're new Pipeline there
SegmentAndClustering<pcl::PointXYZRGBA, Descriptor>* cp= new SegmentAndClustering<pcl::PointXYZRGBA, Descriptor>(new ShapeGlobalXtion<PointType, Descriptor>("bob1"), new ShapeGlobalXtion<PointType, Descriptor>("bob2"));

std::string frame;

bool search_status=false;

bool flagy=true;

/*******Function********/



/**************µCALLBACKS************************/

//Service Server Old not useful anymore

/*bool getPose(stalker::getobject::Request  &req, stalker::getobject::Response &res){
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
		//Calculate the pose compared the base link... 
		stalker::calculatePose(cp->getRoto()[0], pose_stamped.pose ,pose_stamped.pose );
		
		pose_stamped.header.stamp=ros::Time::now();
		
		res.gotObject=true;
		res.pose=pose_stamped;
		res.who="Stalker";		
	}	
	
}*/


//TODO Service CLIENT : 

//TODO A TESTER PARCE QUE CODER EN 5 SECONDES.

void service_client(ros::ServiceClient& client, tf::TransformListener* listener, std::string& to, int id){
	
	int i=0;
	geometry_msgs::PoseStamped pose_stamped;
	
	pose_stamped.header.frame_id=frame;
	
	pose_stamped.pose.position.x=0;
	pose_stamped.pose.position.y=0;
	pose_stamped.pose.position.z=0;
	
	pose_stamped.pose.orientation.x=0;
	pose_stamped.pose.orientation.y=0;
	pose_stamped.pose.orientation.z=0;
	pose_stamped.pose.orientation.w=1;
	
	stalker::sentobject srv;

	srv.response.keepsearching=true;
	
	//while(srv.response.keepsearching && i<cp->getRoto().size() )
	while(srv.response.keepsearching && i<1 ){
		//No need to actually do the conversion... We can send any frame id to move_base later
		//stalker::calculatePoseBaseLink(cp->getRoto()[i], pose_stamped.pose ,pose_stamped.pose, *listener, to );
		stalker::calculatePose(cp->getRoto()[i], pose_stamped.pose ,pose_stamped.pose);
		pose_stamped.header.stamp=ros::Time::now();
		
		
		
		srv.request.gotObject=true;
		srv.request.pose = pose_stamped;
		srv.request.robot_id=id;
		if (client.call(srv))
		{
			ROS_INFO("Search done ");
			std::cout << "response searching : "<<srv.response.keepsearching<<std::endl;
		}
		else
		{
			ROS_ERROR("Failed to call service searching state");
			//return 1;
		}
		i++;
		std::cout <<"Looping for the service of searching "<<std::endl;
	}
	//TODO
	if(srv.response.keepsearching==false){
		//Put flagy to false when we found a good object
		if( search_status==true){
			flagy=false;
			std::cout<<"search status "<<search_status<<std::endl;
		}
	}
	else if(i>=cp->getRoto().size()){
		std::cout <<"All position and object in the camera vision have been found"<<std::endl;
	}
	else{
		std::cout <<"No idea what happened"<<std::endl;
	}
	
}


void search_callback(const std_msgs::Bool::ConstPtr& boolean){
	//std::cout << "Starting search" << std::endl;
	search_status=boolean->data;
	//Put flagy back to true at the end of the searching state
	if(boolean->data==false){
		flagy=true;
	}
}


void mainCall(const sensor_msgs::PointCloud2ConstPtr& cloudy, ros::Time& timestamp, Main<PointType, Descriptor >* main, ros::Publisher& pose_pub, ros::ServiceClient& client, tf::TransformListener* listener, std::string& to, int id){
	
	std::cout<<"************************************* Got an image from the main source********************************"<<std::endl;
	std::cout << "flags : "<<search_status<<" "<<flagy<<std::endl;
	if(search_status==true && flagy==true){
		std::cout<<"************************************* We need to search said Mother Brain********************************"<<std::endl;

		if(main->gotModel()){
			std::cout<<"Got model"<<std::endl;
			 
			sensor_msgs::PointCloud2 pointcloud;
			pointcloud.header.frame_id="/map";
			stalker::cutPointCloudForMap(*cloudy, pointcloud, *listener);
					
			pcl::PointCloud<PointType>::Ptr _scene(new pcl::PointCloud<PointType>() );
			_scene->is_dense=false;
			pcl::fromROSMsg(pointcloud, *_scene);
			
			stalker::passThrough<PointType>(_scene, _scene, "x", -10, 10);
			stalker::passThrough<PointType>(_scene, _scene, "y", -10, 10);
			
			
			main->doWork(_scene);
			
			
			if(main->foundObject()){
			//publish the new bouding box
				//pose_pub.publish<>();
				frame=pointcloud.header.frame_id;
				service_client(client, listener, to, id);
			}
			
		}
		
		timestamp=ros::Time::now();
	}
	
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
	ros::Subscriber searching_state;
	
	
	ros::Publisher pose_pub;
	ros::Publisher newBB_pub;
	
	tf::TransformListener listener(ros::Duration(10));
	
	//Service
	//ros::ServiceServer service = my_node.advertiseService("/getObject", getPose);
	ros::ServiceClient client = my_node.serviceClient<stalker::sentobject>("/sentobject");
	ROS_INFO("Ready to give the pose");
	
	ros::Timer bomb;
	ros::Time timeStamp=ros::Time::now();
	
	pcl::PointCloud<PointType>::Ptr cloudy_save;
	
	Main<PointType, Descriptor > main;
	
	main.setPipeline(cp);
	ROS_INFO("Going all in");
	/*******PARAMETERS****************/
	
	std::string path2model="/home/robot/catkin_ws/src/Stalker/src/Test/milk.pcd";
	std::string where2read="camera/depth/points_xyzrgb";
	std::string base="base_frame_id";
	std::string id_topic="id";
	std::string to;
	int id;
	priv_node.param<std::string>(base, to, "base_link");
	priv_node.param<int>(id_topic, id, 0);

	

	//priv_node.param<std::string>("/model", path2model, "none");
	ROS_INFO("Going for the variables");
	//main.setResolution(true);
	/* SPIN IMAGE*/
	//cp->setPostProcICPThresh(1e-7);
	cp->getObject()->setRadiusDescriptorsEffective(0.05);
	cp->getScene()->setRadiusDescriptorsEffective(0.05);
	cp->getObject()->setSamplingSizeEffective(0.01);
	cp->getScene()->setSamplingSizeEffective(0.01);
	ROS_INFO("Did all the variables");
	/*SHOT*/	
	/*cp->setPostProcICPThresh(1e-7);
	//cp->getObject()->setRadiusDescriptorsEffective(0.03);
	//cp->getScene()->setRadiusDescriptorsEffective(0.05);
	cp->getObject()->setSamplingSizeEffective(0.005);
	cp->getScene()->setSamplingSizeEffective(0.005);*/
	
	//cp->setAlwaysSeeBest();
	//cp->useGeometricConsistency();
	
	/*****************************************/
	
	/***********MODEL****************/
	if(path2model.compare("none")==0){
		model_sub = my_node.subscribe<sensor_msgs::PointCloud2> ("/model", 1, &Main<PointType, Descriptor >::loadModel, &main);
	}
	else{
		std::cout << "Loading MODEL"<<std::endl;
		pcl::PointCloud<PointType>::Ptr model_pc (new pcl::PointCloud<PointType>);
		pcl::io::loadPCDFile (path2model, *model_pc);
		main.loadModel(model_pc);
	}
	ROS_INFO("Loaded the model");
	
	/***********Mother Brain Subscriber*********/
	searching_state = my_node.subscribe<std_msgs::Bool> ("/search_state", 1, search_callback);

	/***********CAMERA IMAGE*********/
	pointcloud_sub = my_node.subscribe<sensor_msgs::PointCloud2> (where2read, 1, boost::bind(mainCall, _1, timeStamp, &main, pose_pub, client, &listener, to, id) );


	while(ros::ok()){		
		
		
		ros::spinOnce();
	}

}
