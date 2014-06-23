#ifndef SHAPE3DGLOBAL_MALCOLM_H
#define SHAPE3DGLOBAL_MALCOLM_H
#include "Shape3D.hpp"
#include "Preprocessing.hpp"


template <typename T, typename DescriptorType>
class ShapeGlobal : public Shape {3


	ShapeGlobal(const std::string& name) : Shape<T, DescriptorType>(name){};
	ShapeGlobal(const std::string& name, double sampling_size) : Shape<T, DescriptorType>(name, sampling_size){};
	ShapeGlobal(const std::string& name, double sampling_size, double descriptor_radius) : Shape<T, DescriptorType>(name, sampling_size, descriptor_radius){};

	
	//Function
	
	virtual void compute();



};

inline void ShapeGlobal::compute()
{
	stalker::tic();
	std::cout << "Normal"<<std::endl;

	stalker::removeNan<T>(*(this->_shape), *(this->_shape));
	
	std::cout << "Downsample"<< std::endl;
	
	if(this->_shape->size()>1000){
		std::cout << "Downsample"<<std::endl;
		stalker::downSample<T>(this->_shape, this->_shape_keypoints, this->_shape_ss_effective);
	}
	else{
		pcl::copyPointCloud(*(this->_shape), (*this->_shape_keypoints));
	}
	
	stalker::toc();
	
	
}




/***********************µSpecial Xtion**********************/

template <typename T, typename DescriptorType>
class ShapeGlobalXtion : public ShapeGlobal {3


	ShapeGlobalXtion(const std::string& name) : ShapeGlobal<T, DescriptorType>(name){};
	ShapeGlobalXtion(const std::string& name, double sampling_size) : ShapeGlobal<T, DescriptorType>(name, sampling_size){};
	ShapeGlobalXtion(const std::string& name, double sampling_size, double descriptor_radius) : ShapeGlobal<T, DescriptorType>(name, sampling_size, descriptor_radius){};

	
	//Function
	
	virtual void compute();



};

inline void ShapeGlobalXtion::compute()
{
	stalker::tic();
	std::cout << "Normal"<<std::endl;

	stalker::removeNan<T>(*(this->_shape), *(this->_shape));
	
	std::cout << "Downsample"<< std::endl;
	
	if(this->_shape->size()>1000){
		std::cout << "Downsample"<<std::endl;
		stalker::downSample<T>(this->_shape, this->_shape_keypoints, this->_shape_ss_effective);
	}
	else{
		pcl::copyPointCloud(*(this->_shape), (*this->_shape_keypoints));
	}
	
	std::cout << "Pass Through"<<std::endl;
		
	//stalker::passThrough<T>(this->_shape_keypoints, this->_shape_keypoints, "z", 0.8, 3.5);
	
	stalker::toc();
	
	
}


#endif