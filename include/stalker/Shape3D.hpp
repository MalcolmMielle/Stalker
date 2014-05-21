#ifndef SHAPE3D_MALCOLM_H
#define SHAPE3D_MALCOLM_H

//Hello Albert
#include <exception>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <deque>
#include "pcl_ros/point_cloud.h"
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <exception>
#include "Gui.hpp"

//#define DescriptorType pcl::SHOT352
#define NormalType pcl::Normal 
#define RFType pcl::ReferenceFrame 

/*template<typename T, typename DescriptorType>
class Gui;*/


template <typename T, typename DescriptorType>
class Shape{

	protected :
	std::string _id;
	typename pcl::PointCloud<T>::Ptr _shape; //model
	typename pcl::PointCloud<T>::Ptr _shape_keypoints; //model_keypoint
	pcl::PointCloud<NormalType>::Ptr _shape_normals;	//Point cloud of Normals
	typename pcl::PointCloud<DescriptorType>::Ptr _desc;	 //Point Cloud of Descriptors
	double _descrRad; //Radius given by the user to calculate the descriptors
	double _descrRad_effective; //Radius actually used in the calculation. Useful in case we need resolution invariance.
	double _shape_ss; //shape_sampling size given by the user used to downsample
	double _shape_ss_effective; //shape_sampling actually used in the calculation to downsample
	bool resol_state; //Indicate if we want to use the resolution invariance
	double _resolution; //Value of the resolution
	
	public :
	Shape(const std::string& name) : _id(name), _shape(new pcl::PointCloud<T>()),_shape_keypoints(new pcl::PointCloud<T>()),_shape_normals(new pcl::PointCloud<NormalType>()), _desc(new pcl::PointCloud<DescriptorType>()), _descrRad(0.02),_descrRad_effective(0.02), _shape_ss (0.01), _shape_ss_effective(0.01), resol_state(false),_resolution(0){};
	
	Shape(const std::string& name, double sampling_size) : _id(name), _shape(new pcl::PointCloud<T>()),_shape_keypoints(new pcl::PointCloud<T>()),_shape_normals(new pcl::PointCloud<NormalType>()), _desc(new pcl::PointCloud<DescriptorType>()), _descrRad(0.02),_descrRad_effective(0.02), _shape_ss (sampling_size), _shape_ss_effective(sampling_size), resol_state(false),_resolution(0){};
	
	Shape(const std::string& name, double sampling_size, double descriptor_radius) : _id(name), _shape(new pcl::PointCloud<T>()),_shape_keypoints(new pcl::PointCloud<T>()),_shape_normals(new pcl::PointCloud<NormalType>()), _desc(new pcl::PointCloud<DescriptorType>()), _descrRad(descriptor_radius), _descrRad_effective(descriptor_radius), _shape_ss (sampling_size), _shape_ss_effective(sampling_size), resol_state(false),_resolution(0){};

	virtual ~Shape(){
		std::cout<<"delete shape "<<_id <<std::endl;
	};
	//Accesseurs
	virtual const typename pcl::PointCloud<T>::Ptr& getCloud(){return _shape;}
	virtual const typename pcl::PointCloud<T>::Ptr& getKeypoints(){return _shape_keypoints;}
	virtual const typename pcl::PointCloud<NormalType>::Ptr& getNormals(){return _shape_normals;}
	virtual typename pcl::PointCloud<DescriptorType>::Ptr& getDescr(){return _desc;}
	virtual double getRadius(){return _descrRad;}
	virtual double getSamplingSize(){return _shape_ss;}
	virtual const std::string& getName(){return _id;}
	virtual double getResolutionState(){return resol_state;}
	virtual double getResolution(){return _resolution;}
	
	virtual void set(typename pcl::PointCloud<T>::Ptr& p){_shape=p;}
	virtual void setRadius(double r){_descrRad=r;}
	virtual void setSamplingSize(double r){_shape_ss=r;}
	virtual void setDescriptors(typename pcl::PointCloud<DescriptorType>::Ptr& desc){_desc=desc;}
	virtual void setNormals(pcl::PointCloud<NormalType>::Ptr& normal){_shape_normals=normal;}
	virtual void setName(std::string& name){_id=name;}
	virtual void setResolutionState(bool b){ resol_state=b;}
	virtual void setResolution(double r){_resolution=r;}
	//update Shape state
	
	virtual void compute()=0;
	
	virtual void resolutionInvariance();
	
	//Load a model
	virtual void update(typename pcl::PointCloud<T>::Ptr& p);
	virtual bool loadMesh(const std::string& path);
	virtual bool saveMesh();
	
	//Print interface
	virtual void addPrint(Gui<T, DescriptorType>& gui)=0;
	virtual void printupdate(Gui<T, DescriptorType>& gui)=0;
	virtual void remove(Gui<T, DescriptorType>& gui)=0;
};

template <typename T, typename DescriptorType>
inline void Shape<T, DescriptorType>::addPrint(Gui<T, DescriptorType>& gui){
		std::cout<<"Je suis une shape"<<std::endl;
	
		try{
			throw std::invalid_argument("you can't use a shape because it's abstract");

		}
		catch(std::exception const& e){
			std::cerr << "you can't use a shape in the GUI because it's abstract : " << e.what() << std::endl;	
		}
}

template <typename T, typename DescriptorType>
inline void Shape<T, DescriptorType>::update(typename pcl::PointCloud<T>::Ptr& p){
	//Clustering pipe
	this->_shape=p;
	
	compute();
}

template <typename T, typename DescriptorType>
inline bool Shape<T, DescriptorType>::saveMesh(){
	int a = pcl::io::savePCDFileBinary("view", *_shape);
	return true;
}

template <typename T, typename DescriptorType>
inline bool Shape<T, DescriptorType>::loadMesh(const std::string& path){
	if (pcl::io::loadPCDFile (path, *_shape) < 0)
	{
		std::cout << "Error loading model cloud." << std::endl;
		return false;
	}
	return true;
}

template <typename T, typename DescriptorType>
inline void Shape<T, DescriptorType>::resolutionInvariance(){

	if (_resolution != 0.0f)
	{
		this->_shape_ss_effective   = _shape_ss* _resolution;
		//rf_rad_     *= resolution;
		this->_descrRad_effective  = _descrRad* _resolution;
		//this->cg_size_    *= resolution;
	}

}

#endif
