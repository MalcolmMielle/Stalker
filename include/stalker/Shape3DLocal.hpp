#ifndef SHAPE3DLOCAL_MALCOLM_H
#define SHAPE3DLOCAL_MALCOLM_H

#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/spin_image.h>
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
		pcl::SHOTEstimationOMP<T, NormalType, DescriptorType> descr_est;
		descr_est.setRadiusSearch (this->_descrRad);
		descr_est.setInputCloud (this->_shape_keypoints);
		descr_est.setInputNormals (this->_shape_normals);
		descr_est.setSearchSurface (this->_shape);
		std::cout<<"Computin"<<std::endl;
		descr_est.compute (*(this->_desc)); //pointer of desc
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
	
};


template <typename T>
class ShapeLocalBase<T,  pcl::Histogram<153> > : public Shape<T,  pcl::Histogram<153> >{
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
		descr_est.setInputCloud (this->_shape_keypoints);
		descr_est.setInputNormals (this->_shape_normals);
		descr_est.setSearchSurface (this->_shape);
		std::cout<<"Computin"<<std::endl;
		descr_est.compute (*(this->_desc)); //pointer of desc
	}
	
};

//FPFH =>pcl::FPFHSignature33

/*#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>

{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

  ... read, pass in or create a point cloud with normals ...
  ... (note: you can create a single PointCloud<PointNormal> if you want) ...

  // Create the FPFH estimation class, and pass the input dataset+normals to it
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud (cloud);
  fpfh.setInputNormals (normals);
  // alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

  // Create an empty kdtree representation, and pass it to the FPFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ>);

  fpfh.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

  // Use all neighbors in a sphere of radius 5cm
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  fpfh.setRadiusSearch (0.05);

  for (int i = 0; i < normals->points.size(); i++)
{
  if (!pcl::isFinite<pcl::Normal>(normals->points[i]))
  {
    PCL_WARN("normals[%d] is not finite\n", i);
  }
}
  
  
  // Compute the features
  fpfh.compute (*fpfhs);

  // fpfhs->points.size () should have the same size as the input cloud->points.size ()*
}*/


/***************CLASS FINAL**********/

template <typename T, typename DescriptorType>
class ShapeLocal : public ShapeLocalBase<T, DescriptorType> {
	private :

	double _k; //Normal estimation diameter.	
	//Preprocessing<T>* _prep;
	
	public : 	
	ShapeLocal(const std::string& name) : 
	ShapeLocalBase<T, DescriptorType>(name), _k(10){};
	
	ShapeLocal(const std::string& name, double sampling_size) : 
	ShapeLocalBase<T, DescriptorType>(name, sampling_size), _k(10){};
	
	ShapeLocal(const std::string& name, double sampling_size, double descriptor_radius) : 
	ShapeLocalBase<T, DescriptorType>(name, sampling_size, descriptor_radius), _k(10){};


	virtual void setNormalEstimDiameter(double k){_k=k;}
	virtual double getNormalEstimDiameter(){return _k;}

	//update Shape state
	
	virtual void compute();
	//Load a model
	
	virtual void estimNormal();

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



template <typename T, typename DescriptorType>
inline void ShapeLocal<T, DescriptorType>::compute(){
	stalker::tic();
	std::cout << "Normal"<<std::endl;
	if(this->resol_state==true){
		this->resolutionInvariance();
	}

	stalker::removeNan<T>(*(this->_shape), *(this->_shape));
	this->estimNormal();
	std::cout << "Downsample"<<std::endl;
	
	stalker::downSample<T>(this->_shape, this->_shape_keypoints, this->_shape_ss_effective);
	
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

//PROUBLEM ICI
template <typename T, typename DescriptorType>
inline void ShapeLocal<T, DescriptorType>::estimNormal(){
	
	pcl::NormalEstimationOMP<T, NormalType> norm_est2;
	norm_est2.setKSearch (_k);
	std::cout<<"Input cloud given"<<std::endl;
	norm_est2.setInputCloud (this->_shape);
	std::cout<<"compte the normals"<<std::endl;
	try{
		if(this->_shape->is_dense==true){
			norm_est2.compute (*(this->_shape_normals));
		}
		else{
			throw std::invalid_argument("not dense");
		}
	}
	catch(std::exception const& e){
		std::cerr << "ERREUR SHAPE is not dense : " << e.what() << std::endl;
		exit(0);
	}
		

}

/*************SHAPE COLOR******************/




#endif
