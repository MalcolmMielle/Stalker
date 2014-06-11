#ifndef SHAPE3DLOCAL_MALCOLM_H
#define SHAPE3DLOCAL_MALCOLM_H

#include <pcl/features/shot_omp.h>
#include <pcl/features/spin_image.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include "Shape3D.hpp"
//#include "PreprocessingSimple.hpp"
#include "Preprocessing.hpp"

//#include "Gui.hpp"

/******************HOW YOU'RE SUPPOSED TO CODE THAT**********
template< typename _T, size_t num >
struct FooFuncBase {
    void Func() {
        printf("Hello world!");
    }
};

template< typename _T >
struct FooFuncBase< _T, 1 > {
    void Func() {
        printf("Hi!");
    }
};

template< typename _T, size_t num >
struct Foo : public FooFuncBase< _T, num > {
  void OtherFuncWhoseImplementationDoesNotDependOnNum() {
    ...
  }
};

************************************************************/

/*******TEMPLATE SPECIALIZATION**********/

template <typename T, typename DescriptorType>
class ShapeLocalBase : public Shape<T, DescriptorType>{
	public :
	
	ShapeLocalBase(const std::string& name) : Shape<T, DescriptorType>(name){};
	ShapeLocalBase(const std::string& name, double sampling_size) : Shape<T, DescriptorType>(name, sampling_size){};
	ShapeLocalBase(const std::string& name, double sampling_size, double descriptor_radius) : Shape<T, DescriptorType>(name, sampling_size, descriptor_radius){};
	
	/***/
	
	virtual void computeDescriptors(){
		std::cout<<"Default descriptor"<<std::endl;
		pcl::SHOTEstimationOMP<T, NormalType, DescriptorType> descr_est;
		descr_est.setRadiusSearch (this->_descrRad);
		descr_est.setInputCloud (this->_shape_keypoints);
		descr_est.setInputNormals (this->_shape_normals);
		descr_est.setSearchSurface (this->_shape);
		std::cout<<"Computin"<<std::endl;
		descr_est.compute (*(this->_desc)); //pointer of desc
	}
	
	virtual void compute(){
		stalker::tic();
		std::cout << "Normal"<<std::endl;

		stalker::removeNan<T>(*(this->_shape), *(this->_shape));
		
		
		stalker::estimNormal<T, NormalType>(this->_shape, this->_shape_normals, this->_k);
		
		if(this->_shape->size()>1000){
			std::cout << "Downsample"<<std::endl;
			stalker::downSample<T>(this->_shape, this->_shape_keypoints, this->_shape_ss_effective);
		}
		else{
			pcl::copyPointCloud(*(this->_shape), (*this->_shape_keypoints));
		}
		
		
		std::cout << "Pass Through"<<std::endl;
		
		//stalker::passThrough<T>(this->_shape_keypoints, this->_shape_keypoints, "z", 0.8, 3.5);
		
		std::cout << "Outlier removal"<<std::endl;
		
		//stalker::statisticalOutilerRemoval<T>(this->_shape_keypoints, this->_shape_keypoints, 50, 1.0);
		
		std::cout << "compute descriptors"<<std::endl;
		
		this->computeDescriptors();
		std::cout<<std::endl<<std::endl;
		std::cout<<"Shape ";
		stalker::toc();
	}
		
};


template <typename T>
class ShapeLocalBase<T, pcl::SHOT1344> : public Shape<T, pcl::SHOT1344>{
	public :
		
	ShapeLocalBase(const std::string& name) : Shape<T, pcl::SHOT1344>(name){};
	ShapeLocalBase(const std::string& name, double sampling_size) : Shape<T, pcl::SHOT1344>(name, sampling_size){};
	ShapeLocalBase(const std::string& name, double sampling_size, double descriptor_radius) : Shape<T, pcl::SHOT1344>(name, sampling_size, descriptor_radius){};
	
	/***/
	
	virtual void computeDescriptors(){
		// USING THE COLORS
		std::cout << "Color Descriptors !"<<std::endl;
		pcl::SHOTColorEstimationOMP<T, NormalType, pcl::SHOT1344> descr_est;
		descr_est.setRadiusSearch (this->_descrRad_effective);
		descr_est.setInputCloud (this->_shape_keypoints);
		descr_est.setInputNormals (this->_shape_normals);
		descr_est.setSearchSurface (this->_shape);
		std::cout<<"Computin"<<std::endl;
		descr_est.compute (*(this->_desc)); //pointer of desc
	}
	
	virtual void compute(){
		stalker::tic();
		std::cout << "Normal"<<std::endl;

		stalker::removeNan<T>(*(this->_shape), *(this->_shape));
		stalker::estimNormal<T, NormalType>(this->_shape, this->_shape_normals, this->_k);
		
		if(this->_shape->size()>1000){
			std::cout << "Downsample"<<std::endl;
			stalker::downSample<T>(this->_shape, this->_shape_keypoints, this->_shape_ss_effective);
		}
		else{
			pcl::copyPointCloud(*(this->_shape), (*this->_shape_keypoints));
		}
		
		//std::cout << "Pass Through"<<std::endl;		
		//stalker::passThrough<T>(this->_shape_keypoints, this->_shape_keypoints, "z", 0.8, 3.5);
		
		std::cout << "compute descriptors"<<std::endl;
		
		this->computeDescriptors();
		std::cout<<std::endl<<std::endl;
		std::cout<<"Shape ";
		stalker::toc();
	}
	
};


template <typename T>
class ShapeLocalBase<T, pcl::Histogram<153> > : public Shape<T,  pcl::Histogram<153> >{
	public :
		
	ShapeLocalBase(const std::string& name) : Shape<T, pcl::Histogram<153> >(name){};
	ShapeLocalBase(const std::string& name, double sampling_size) : Shape<T, pcl::Histogram<153> >(name, sampling_size){};
	ShapeLocalBase(const std::string& name, double sampling_size, double descriptor_radius) : Shape<T, pcl::Histogram<153> >(name, sampling_size, descriptor_radius){};
	
	/***/
	
	virtual void computeDescriptors(){
		// USING THE COLORS
		std::cout << "Spin Image Descriptors !"<<std::endl;
		pcl::SpinImageEstimation<T, NormalType, pcl::Histogram<153> > descr_est;
		descr_est.setRadiusSearch (this->_descrRad_effective);
		descr_est.setInputCloud (this->_shape_keypoints); //Consider everyting
		descr_est.setInputNormals (this->_shape_normals);
		descr_est.setSearchSurface (this->_shape);
		std::cout<<"Computin"<<std::endl;
		descr_est.compute (*(this->_desc)); //pointer of desc
	}
	
	virtual void compute(){
		stalker::tic();
		std::cout << "RemoveNan"<<std::endl;

		stalker::removeNan<T>(*(this->_shape), *(this->_shape));
		
		if(this->_shape->size()>1000){
			std::cout << "Downsample"<<std::endl;
			stalker::downSample<T>(this->_shape, this->_shape_keypoints, this->_shape_ss_effective);
		}
		else{
			pcl::copyPointCloud(*(this->_shape), (*this->_shape_keypoints));
		}

		std::cout << "Normal "<<std::endl;
		stalker::estimNormal<T, NormalType>(this->_shape_keypoints, this->_shape_normals, this->_k);
		
		//std::cout << "Pass Through"<<std::endl;
		
		//stalker::passThrough<T>(this->_shape_keypoints, this->_shape_keypoints, "z", 0.8, 3.5);
		
		std::cout << "compute descriptors"<<std::endl;

		this->computeDescriptors();

		//Spin image need the same number of normal than keypoints. Thus we calculate the normals after the downsample and from the keypoints Point Coud.
		stalker::estimNormal<T, NormalType>(this->_shape, this->_shape_normals, this->_k);
		
		std::cout<<std::endl<<std::endl;
		std::cout<<"Shape ";
		stalker::toc();
	}
	
};

//FPFH got a field named histogram[33]
template <typename T>
class ShapeLocalBase<T, pcl::FPFHSignature33 > : public Shape<T,  pcl::FPFHSignature33 >{
	public :
		
	ShapeLocalBase(const std::string& name) : Shape<T, pcl::FPFHSignature33 >(name){};
	ShapeLocalBase(const std::string& name, double sampling_size) : Shape<T, pcl::FPFHSignature33 >(name, sampling_size){};
	ShapeLocalBase(const std::string& name, double sampling_size, double descriptor_radius) : Shape<T, pcl::FPFHSignature33 >(name, sampling_size, descriptor_radius){};
	
	/***/
	
	virtual void computeDescriptors(){
		// USING THE COLORS
		std::cout << "Spin Image Descriptors !"<<std::endl;
		pcl::FPFHEstimationOMP<T, NormalType, pcl::FPFHSignature33 > fpfh;
		fpfh.setRadiusSearch (this->_descrRad_effective);
		fpfh.setInputCloud (this->_shape_keypoints);
		fpfh.setInputNormals (this->_shape_normals);
		// Create an empty kdtree representation, and pass it to the FPFH estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		typename pcl::search::KdTree<T>::Ptr tree (new pcl::search::KdTree<T>);
		fpfh.setSearchMethod (tree);
		
		fpfh.setSearchSurface (this->_shape);
		std::cout<<"Computin"<<std::endl;
		
		for (int i = 0; i < this->_shape_normals->points.size(); i++)
		{
			if (!pcl::isFinite<pcl::Normal>(this->_shape_normals->points[i]))
			{
				PCL_WARN("normals[%d] is not finite\n", i);
			}
		}
		
		fpfh.compute (*(this->_desc)); //pointer of desc
	}
	
	virtual void compute(){
		stalker::tic();
		std::cout << "Normal"<<std::endl;


		stalker::removeNan<T>(*(this->_shape), *(this->_shape));	
		stalker::estimNormal<T, NormalType>(this->_shape, this->_shape_normals, this->_k);
		
		if(this->_shape->size()>1000){
			std::cout << "Downsample"<<std::endl;
			stalker::downSample<T>(this->_shape, this->_shape_keypoints, this->_shape_ss_effective);
		}
		else{
			pcl::copyPointCloud(*(this->_shape), (*this->_shape_keypoints));
		}
				
		//std::cout << "Pass Through"<<std::endl;	
		//stalker::passThrough<T>(this->_shape_keypoints, this->_shape_keypoints, "z", 0.8, 3.5);
		
		std::cout << "compute descriptors"<<std::endl;
		
		this->computeDescriptors();
		std::cout<<std::endl<<std::endl;
		std::cout<<"Shape ";
		stalker::toc();
	}
	
};

/***************CLASS FINAL**********/

template <typename T, typename DescriptorType>
class ShapeLocal : public ShapeLocalBase<T, DescriptorType> {
	private :


	//Preprocessing<T>* _prep;
	
	public : 	
	ShapeLocal(const std::string& name) : 
	ShapeLocalBase<T, DescriptorType>(name){};
	
	ShapeLocal(const std::string& name, double sampling_size) : 
	ShapeLocalBase<T, DescriptorType>(name, sampling_size){};
	
	ShapeLocal(const std::string& name, double sampling_size, double descriptor_radius) : 
	ShapeLocalBase<T, DescriptorType>(name, sampling_size, descriptor_radius){};

	//update Shape state
	//Load a model

	//PRINT
	//Print interface
	virtual void addPrint(Gui<T, DescriptorType>& gui){
		std::cout<<"Je suis une shape locale"<<std::endl;
		gui.add(*this);
	}
	
	virtual void printupdate(Gui<T, DescriptorType>& gui){
		gui.update(*this);
	}

	virtual void remove(Gui<T, DescriptorType>& gui){
		gui.remove(*this);
	}

};




#endif
