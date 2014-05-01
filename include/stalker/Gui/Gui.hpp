#ifndef GUI_MALCOLM_H
#define GUI_MALCOLM_H

// PCL specific includes
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

template <typename T>
class Gui{
	public : 
	pcl::visualization::PCLVisualizer* viewer;
	
	public : 
	Gui() : viewer(new pcl::visualization::PCLVisualizer ("3D Viewer")){
		std::cout<<"buiding the gui"<<std::endl;
		viewer->setBackgroundColor (0, 0, 0);
		viewer->addCoordinateSystem (1.0);
		viewer->initCameraParameters ();
		};
	~Gui(){ 
		std::cout<<"deleting the gui"<<std::endl;
		delete viewer;}
	
	virtual void add(typename pcl::PointCloud<T>::Ptr cloud, std::string name){
		viewer->addPointCloud<T> (cloud, name);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name);
	}		
	
	virtual void update(typename pcl::PointCloud<T>::Ptr cloud, std::string name){
		viewer->updatePointCloud(cloud, name);
	}
	
	virtual void show(){
		viewer->spinOnce (100);
	}


};

#endif
