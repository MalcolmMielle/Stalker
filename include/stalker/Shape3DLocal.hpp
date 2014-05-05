#ifndef SHAPE3DLOCAL_MALCOLM_H
#define SHAPE3DLOCAL_MALCOLM_H

#include "Shape3D.hpp"

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
		descr_est.setRadiusSearch (this->_descrRad);
		descr_est.setInputCloud (this->_shape_keypoints);
		descr_est.setInputNormals (this->_shape_normals);
		descr_est.setSearchSurface (this->_shape);
		std::cout<<"Computin"<<std::endl;
		descr_est.compute (*(this->_desc)); //pointer of desc
	}
	
};


/***************CLASS FINAL**********/

template <typename T, typename DescriptorType>
class ShapeLocal : public ShapeLocalBase<T, DescriptorType> {
	public :
	bool resol;	
	int _k; //Normal estimation diameter.
	ShapeLocal(const std::string& name) : ShapeLocalBase<T, DescriptorType>(name), resol(false), _k(10){};
	ShapeLocal(const std::string& name, double sampling_size) : ShapeLocalBase<T, DescriptorType>(name, sampling_size), resol(false), _k(10){};
	ShapeLocal(const std::string& name, double sampling_size, double descriptor_radius) : ShapeLocalBase<T, DescriptorType>(name, sampling_size, descriptor_radius), resol(false), _k(10){};

	virtual void setNormalEstimator(int k){
		_k=k;
	}
	//update Shape state
	
	virtual void compute();
	//Load a model
	
	
	virtual double computeCloudResolution(typename pcl::PointCloud<T>::Ptr cloud);
	//TODO : move to Preprocessing with downsample !
	virtual void estimNormal();
	virtual void downsample();
	virtual void resolutionInvariance();
};



template <typename T, typename DescriptorType>
inline void ShapeLocal<T, DescriptorType>::compute(){
	if(resol==true){
		std::cout << "Resolution"<<std::endl;
		resolutionInvariance();
	}
		std::cout << "Normal"<<std::endl;
	this->estimNormal();
		std::cout << "DownSample"<<std::endl;
	this->downsample();
		std::cout << "Descriptors"<<std::endl;
	this->computeDescriptors();
}

//PROUBLEM ICI
template <typename T, typename DescriptorType>
inline void ShapeLocal<T, DescriptorType>::estimNormal(){
	
	pcl::NormalEstimationOMP<T, NormalType> norm_est2;
	norm_est2.setKSearch (_k);
	norm_est2.setInputCloud (this->_shape);
	norm_est2.compute (*(this->_shape_normals));

}


template <typename T, typename DescriptorType>
inline void ShapeLocal<T, DescriptorType>::downsample(){
	pcl::PointCloud<int> sampled_indices;

	pcl::UniformSampling<T> uniform_sampling;
	uniform_sampling.setInputCloud (this->_shape);
	uniform_sampling.setRadiusSearch (this->_shape_ss);
	uniform_sampling.compute (sampled_indices);
	pcl::copyPointCloud (*(this->_shape), sampled_indices.points, *(this->_shape_keypoints));
	
	std::cout << "Shape total points: " << this->_shape->size () << "; Selected Keypoints: " << this->_shape_keypoints->size () << std::endl;
}


template <typename T, typename DescriptorType>
inline void ShapeLocal<T, DescriptorType>::resolutionInvariance(){

	float resolution = static_cast<float> (computeCloudResolution (this->_shape));
	if (resolution != 0.0f)
	{
		this->_shape_ss   *= resolution;
		//rf_rad_     *= resolution;
		this->_descrRad  *= resolution;
		//this->cg_size_    *= resolution;
	}

	/*std::cout << "Model resolution:       " << resolution << std::endl;
	std::cout << "Model sampling size:    " << model_ss_ << std::endl;
	std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
	std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
	std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
	std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;*/

}

template <typename T, typename DescriptorType>
inline double ShapeLocal<T, DescriptorType>::computeCloudResolution (typename pcl::PointCloud<T>::Ptr cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices (2);
	std::vector<float> sqr_distances (2);
	pcl::search::KdTree<T> tree;
	tree.setInputCloud (cloud);

	for (size_t i = 0; i < cloud->size (); ++i)
	{
		if (! pcl_isfinite ((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt (sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}


/*************SHAPE COLOR******************/




#endif
