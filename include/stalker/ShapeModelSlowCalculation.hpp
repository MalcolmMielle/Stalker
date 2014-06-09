#include "Shape3DLocal.hpp"

template <typename T, typename DescriptorType>
class ShapeModelSlowCalculation : public ShapeLocal<T, DescriptorType>{

protected : 
	double thresold;
	public : 
	
	ShapeModelSlowCalculation(const std::string& name, double thresold) : ShapeLocal<T, DescriptorType>(name){};
	ShapeModelSlowCalculation(const std::string& name, double sampling_size, double thresold) : ShapeLocal<T, DescriptorType>(name, sampling_size){};
	ShapeModelSlowCalculation(const std::string& name, double sampling_size, double descriptor_radius, double thresold) : ShapeLocal<T, DescriptorType>(name, sampling_size, descriptor_radius){};

	virtual void compute(){
		
		stalker::tic();
		std::cout << "Normal"<<std::endl;
		stalker::removeNan<T>(*(this->_shape), *(this->_shape));
		
		if(this->_shape->size()>1000){
			std::cout << "Downsample"<<std::endl;
			stalker::downSample<T>(this->_shape, this->_shape_keypoints, this->_shape_ss_effective);
			
			stalker::estimNormal<T, NormalType>(this->_shape_keypoints, this->_shape_normals, this->_k);
			std::cout << "Downsample in uniform grid"<<std::endl;
			stalker::downSampleUniformGrid<T>(this->_shape_keypoints, this->_shape_keypoints, thresold);
		}
		else{
			pcl::copyPointCloud(*(this->_shape), (*this->_shape_keypoints));
		}
		
		stalker::estimNormal<T, NormalType>(this->_shape, this->_shape_normals, this->_k);
		//std::cout << "Pass Through"<<std::endl;
		
		//stalker::passThrough<T>(this->_shape_keypoints, this->_shape_keypoints, "z", 0.8, 3.5);
		std::cout << "compute descriptors"<<std::endl;
		this->computeDescriptors();
		std::cout<<std::endl<<std::endl;
		std::cout<<"Shape ";
		stalker::toc();
	}

};




//SPIN IMAGE


template <typename T>
class ShapeModelSlowCalculation<T, pcl::Histogram<153> > : public ShapeLocal<T, pcl::Histogram<153> >{

protected : 
	double thresold;
	public : 
	
	ShapeModelSlowCalculation(const std::string& name, double thresold) : ShapeLocal<T, pcl::Histogram<153> >(name){};
	ShapeModelSlowCalculation(const std::string& name, double sampling_size, double thresold) : ShapeLocal<T, pcl::Histogram<153> >(name, sampling_size){};
	ShapeModelSlowCalculation(const std::string& name, double sampling_size, double descriptor_radius, double thresold) : ShapeLocal<T, pcl::Histogram<153> >(name, sampling_size, descriptor_radius){};

	virtual void compute(){
		
		stalker::tic();
		std::cout << "Normal"<<std::endl;
		stalker::removeNan<T>(*(this->_shape), *(this->_shape));
		
		stalker::estimNormal<T, NormalType>(this->_shape_keypoints, this->_shape_normals, this->_k);
		
		if(this->_shape->size()>1000){
			std::cout << "Downsample"<<std::endl;
			stalker::downSample<T>(this->_shape, this->_shape_keypoints, this->_shape_ss_effective);
				
			std::cout << "Downsample in uniform grid"<<std::endl;
			stalker::downSampleUniformGrid<T>(this->_shape_keypoints, this->_shape_keypoints, thresold);
		}
		else{
			pcl::copyPointCloud(*(this->_shape), (*this->_shape_keypoints));
		}
		//std::cout << "Pass Through"<<std::endl;
		
		//stalker::passThrough<T>(this->_shape_keypoints, this->_shape_keypoints, "z", 0.8, 3.5);
		
		std::cout << "compute descriptors"<<std::endl;
		this->computeDescriptors();
		std::cout<<std::endl<<std::endl;
		
		stalker::estimNormal<T, NormalType>(this->_shape, this->_shape_normals, this->_k);
		std::cout<<"Shape ";
		stalker::toc();
	}

};