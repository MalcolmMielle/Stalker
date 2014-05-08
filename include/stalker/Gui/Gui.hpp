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

template<typename T, typename DescriptorType>
class ShapeLocal;

template<typename T, typename DescriptorType>
class Shape;

template<typename T, typename DescriptorType>
class Pipeline;

template<typename T>
class CorrespGrouping;

#include <pcl/io/pcd_io.h>


template <typename T, typename DescriptorType>
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
	
	virtual void add(Shape<T, DescriptorType>& sh)=0;
	virtual void add(ShapeLocal<T, DescriptorType>& sh)=0;
	virtual void printPipeline(Pipeline<T, DescriptorType>& p)=0;
	virtual void printPipeline(CorrespGrouping<T>& sh)=0;
	/*{
		//sh->addPrint(this);
		add(sh->getCloud(), sh->getName() );
	}*/
	
	virtual void add(typename pcl::PointCloud<T>::Ptr cloud, std::string name)=0;		
	
	virtual void update(typename pcl::PointCloud<T>::Ptr cloud, std::string name)=0;
	virtual void update(Shape<T, DescriptorType>& sh)=0;
	virtual void update(ShapeLocal<T, DescriptorType>& sh)=0;
	
	virtual bool wasStopped(){return viewer->wasStopped ();}
	virtual void show(){
		this->viewer->spinOnce (100);
	}	


};

#endif
