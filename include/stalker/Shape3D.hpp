#ifndef SHAPE3D_MALCOLM_H
#define SHAPE3D_MALCOLM_H

#include <iostream>
#include <stdio.h>
#include <vector>
#include <deque>
#include "pcl_ros/point_cloud.h"
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/io/pcd_io.h>

//#define DescriptorType pcl::SHOT352
#define NormalType pcl::Normal 
#define RFType pcl::ReferenceFrame 

template <typename T, typename DescriptorType>
class Shape{

	protected :
	std::string _id;
	typename pcl::PointCloud<T>::Ptr _shape; //model
	typename pcl::PointCloud<T>::Ptr _shape_keypoints; //model_keypoint
	pcl::PointCloud<NormalType>::Ptr _shape_normals;	
	typename pcl::PointCloud<DescriptorType>::Ptr _desc;	
	double _descrRad;
	double _shape_ss; //shape_sampling size
	
	public :
	Shape(const std::string& name) : _id(name), _shape(new pcl::PointCloud<T>()),_shape_keypoints(new pcl::PointCloud<T>()),_shape_normals(new pcl::PointCloud<NormalType>()), _desc(new pcl::PointCloud<DescriptorType>()), _descrRad(0.02), _shape_ss (0.01){};
	
	Shape(const std::string& name, double sampling_size) : _id(name), _shape(new pcl::PointCloud<T>()),_shape_keypoints(new pcl::PointCloud<T>()),_shape_normals(new pcl::PointCloud<NormalType>()), _desc(new pcl::PointCloud<DescriptorType>()), _descrRad(0.02), _shape_ss (sampling_size){};
	
	Shape(const std::string& name, double sampling_size, double descriptor_radius) : _id(name), _shape(new pcl::PointCloud<T>()),_shape_keypoints(new pcl::PointCloud<T>()),_shape_normals(new pcl::PointCloud<NormalType>()), _desc(new pcl::PointCloud<DescriptorType>()), _descrRad(descriptor_radius), _shape_ss (sampling_size){};

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
	
	virtual void set(typename pcl::PointCloud<T>::Ptr& p){_shape=p;}
	virtual void setRadius(float r){_descrRad=r;}
	virtual void setSamplingSize(float r){_shape_ss=r;}
	virtual void setDescriptors(typename pcl::PointCloud<DescriptorType>::Ptr& desc){_desc=desc;}
	virtual void setNormals(pcl::PointCloud<NormalType>::Ptr& normal){_shape_normals=normal;}
	//update Shape state
	
	virtual void compute()=0;

	//Load a model
	virtual void update(typename pcl::PointCloud<T>::Ptr& p);
	virtual bool loadMesh(std::string path);
	virtual bool saveMesh();

};

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
inline bool Shape<T, DescriptorType>::loadMesh(std::string path){
	if (pcl::io::loadPCDFile (path, *_shape) < 0)
	{
		std::cout << "Error loading model cloud." << std::endl;
		return false;
	}
	return true;
}

#endif
