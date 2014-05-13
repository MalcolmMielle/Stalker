#ifndef SHAPE3DLOCAL_MALCOLM_H
#define SHAPE3DLOCAL_MALCOLM_H

#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/spin_image.h>
#include "Shape3D.hpp"
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
		descr_est.setRadiusSearch (this->_descrRad);
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

	
	/*virtual void addPrint(Gui<T, DescriptorType>& gui){
		std::cout<<"Je suis une shape LOCALE"<<std::endl;
		gui.add(*this);
	}*/
	
	/*virtual void addPrint(Gui<T, DescriptorType>* gui){
		gui->add(this->_shape, this->_id);
		//printKeyPoints(gui);
		
	}
	
	virtual void printKeyPoints(Gui<T, DescriptorType>* gui){
		pcl::visualization::PointCloudColorHandlerCustom<T> scene_keypoints_color_handler (this->_shape_keypoints, 0, 0, 255);
		gui->viewer->addPointCloud (this->_shape_keypoints, scene_keypoints_color_handler, "scene_keypoints");
		gui->viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");
	}*/
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
	std::cout<<"Input cloud given"<<std::endl;
	norm_est2.setInputCloud (this->_shape);
	std::cout<<"compte the normals"<<std::endl;
	try{
		if(this->_shape->is_dense==true){
			std::cout<<"Yay dense :D"<<std::endl;
			norm_est2.compute (*(this->_shape_normals));
		}
		else{
			throw std::invalid_argument("not dense");
		}
	}
	catch(std::exception const& e){
		std::cerr << "ERREUR SHAPE is not dense :( : " << e.what() << std::endl;
		exit(0);
	}
		

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
