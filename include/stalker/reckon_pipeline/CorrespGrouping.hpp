#ifndef CORRESGROUP_RECKON_H
#define CORRESGROUP_RECKON_H

#include "CorrespGroupingBase.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/correspondence.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/features/board.h>
#include <pcl/features/shot_lrf.h>
#include <pcl/recognition/cg/geometric_consistency.h>


template <typename T, typename DescriptorTypes>
class CorrespGrouping : public CorrespGroupingBase<T, DescriptorTypes> {
	protected: 
	//ShapeLocal<T, pcl::SHOT352>* _object; //Don't need because it take a Shape* from main.
	//ShapeLocal<T, pcl::SHOT352>* _scene; // Need to be initialise because it takes a Cloud in argument.
	bool resol;
	
	double _rf_rad; //Referance frame radius default 0.015
	double _cg_size; //Cluster size default 0.01
	double _cg_thresh; //Cluster thressold default 5
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > _rototranslations;
	std::vector<pcl::Correspondences> _clustered_corrs;
	
	public : 
		
	CorrespGrouping(ShapeLocal<T, DescriptorTypes>* object, ShapeLocal<T, DescriptorTypes>* scene) : CorrespGroupingBase<T, DescriptorTypes>(object, scene),/* _object(object), _scene(scene),*/resol(false), _rf_rad(0.015), _cg_size(0.01), _cg_thresh(5.0) {};
	
	virtual void doPipeline();
	//virtual void doPipelineOld();
	
	//new stuff
	virtual void setResol(bool y){resol=y;}
	virtual void setFrameRadius(int rf){_rf_rad=rf;}
	virtual void setClusterSize(int rf){_cg_size=rf;}
	virtual void setClusterThresold(int rf){_cg_thresh=rf;}
	
	virtual int getFrameRadius(){return _rf_rad;}
	virtual int getClusterSize(){return _cg_size;}
	virtual int getClusterThresold(){return _cg_thresh;}
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& getRoto(){return _rototranslations;}
	std::vector<pcl::Correspondences>& getClust(){return _clustered_corrs;}
	
	
	
	virtual void clusteringHough();
	virtual void resolutionInvariance();
	//virtual void estimNormal();
	
	virtual void affiche();
	virtual void printinfo(){
		std::cout<<"INSIDE THE CLASS"<<std::endl;
		std::cout<<"heig "<<this->_scene->getCloud()->height<<" width "<<this->_scene->getCloud()->width<<" size "<< this->_scene->getCloud()->size()<<" dense "<< this->_scene->getCloud()->is_dense<< " Organized "<<this->_scene->getCloud()->isOrganized() <<std::endl;
		
		std::cout<<"INSIDE THE CLASS model"<<std::endl;
		std::cout<<"heig "<<this->_object->getCloud()->height<<" width "<<this->_object->getCloud()->width<<" size "<< this->_object->getCloud()->size()<<" dense "<< this->_object->getCloud()->is_dense<<" Organized "<<this->_object->getCloud()->isOrganized() <<std::endl;
	}
	
	//OVERWRITTEN FUNTION
	virtual void print(Gui<T,DescriptorTypes>* g){
		g->printPipeline((*this));
	}
	
	//OVERWRITTEN FUNTION
	//virtual void setScene(typename pcl::PointCloud<T>::Ptr& obj){this->_scene->set(obj);}
	//virtual void setObject(typename pcl::PointCloud<T>::Ptr& obj){this->_object->set(obj);}

};

/*template <typename T>
inline void CorrespGrouping<T>::doPipelineOld()
{
	std::cout<<"ENTER THE OLD PIPELINE ************************************"<<std::endl;
	if(resol==true){
		std::cout << "Rosulation"<<std::endl;
		resolutionInvariance();
	}
	std::cout<<"Narmooool"<<std::endl;
	this->estimNormal();
		std::cout << "DownSample"<<std::endl;
	this->_object->downsample();
	this->_scene->downsample();
		std::cout << "Descriptors"<<std::endl;
	this->_scene->computeDescriptors();
	this->_object->computeDescriptors();
	
	point2PointCorrespondance();
	clusteringHough();
	
	
}*/

template <typename T, typename DescriptorTypes>
inline void CorrespGrouping<T, DescriptorTypes>::doPipeline()
{
	std::cout<<"ENTER THE PIPELINE ************************************"<<std::endl;
	if(resol==true){
		std::cout << "Rosulation"<<std::endl;
		resolutionInvariance();
	}
	this->point2PointCorrespondance();
	clusteringHough();
}


template <typename T, typename DescriptorTypes>
inline void CorrespGrouping<T, DescriptorTypes>::clusteringHough(){
	    //
	//  Compute (Keypoints) Reference Frames only for Hough
	
	pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
	pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

	//USED THAT IN PCL 1.7
	pcl::BOARDLocalReferenceFrameEstimation<T, NormalType, RFType> rf_est;
	rf_est.setFindHoles (true);
	std::cout <<"Defining the radius at " <<_rf_rad<<std::endl; 
	rf_est.setRadiusSearch (_rf_rad);

	rf_est.setInputCloud (this->_object->getKeypoints());
	rf_est.setInputNormals (this->_object->getNormals());
	rf_est.setSearchSurface (this->_object->getCloud());
	rf_est.compute (*model_rf);

	rf_est.setInputCloud (this->_scene->getKeypoints());
	rf_est.setInputNormals (this->_scene->getNormals());
	rf_est.setSearchSurface (this->_scene->getCloud());
	rf_est.compute (*scene_rf); 

	//  Clustering
	pcl::Hough3DGrouping<T, T, RFType, RFType> clusterer; //undefined reference in .o
	clusterer.setHoughBinSize (_cg_size);
	clusterer.setHoughThreshold (_cg_thresh);
	clusterer.setUseInterpolation (true);
	clusterer.setUseDistanceWeight (false);

	clusterer.setInputCloud (this->_object->getKeypoints());
	clusterer.setInputRf (model_rf);
	clusterer.setSceneCloud (this->_scene->getKeypoints());
	clusterer.setSceneRf (scene_rf);
	clusterer.setModelSceneCorrespondences (this->_model_scene_corrs);

	//clusterer.cluster (clustered_corrs);
	clusterer.recognize (_rototranslations, _clustered_corrs);
	/*
	pcl::GeometricConsistencyGrouping<T, T> gc_clusterer;
	gc_clusterer.setGCSize (_cg_size);
	gc_clusterer.setGCThreshold (_cg_thresh);

	gc_clusterer.setInputCloud (this->_object->getKeypoints());
	gc_clusterer.setSceneCloud (this->_scene->getKeypoints());
	gc_clusterer.setModelSceneCorrespondences (this->_model_scene_corrs);

	//gc_clusterer.cluster (clustered_corrs);
	gc_clusterer.recognize (_rototranslations, _clustered_corrs);*/

}



template <typename T, typename DescriptorTypes>
inline void CorrespGrouping<T, DescriptorTypes>::resolutionInvariance(){

	float resolution = 0;//static_cast<float> (computeCloudResolution (this->_shape));
	//ATTENTION CHECK QUE CA A ETE FAIT CHEZ SHAPE !!!
	if (resol==true)
	{
		//this->_shape_ss   *= resolution;
		this->_rf_rad     *= resolution;
		//this->_descrRad  *= resolution;
		this->_cg_size    *= resolution;
	}

	/*std::cout << "Model resolution:       " << resolution << std::endl;
	std::cout << "Model sampling size:    " << model_ss_ << std::endl;
	std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
	std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
	std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
	std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;*/

}

/*
template <typename PointType>
inline void CorrespGrouping<PointType>::estimNormal()
{
	printinfo();
	
	pcl::NormalEstimationOMP<PointType, NormalType> norm_est2;
	norm_est2.setKSearch (10);
	norm_est2.setInputCloud (this->_object->getCloud());
	norm_est2.compute (*(this->_object->getNormals()));

	norm_est2.setInputCloud (this->_scene->getCloud());
	norm_est2.compute (*(this->_scene->getNormals()));
}*/


template <typename T, typename DescriptorTypes>
inline void CorrespGrouping<T, DescriptorTypes>::affiche()
{
	std::cout << "Model instances found: " << _rototranslations.size () << std::endl;
	for (size_t i = 0; i < _rototranslations.size (); ++i)
	{
		std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
		std::cout << "        Correspondences belonging to this instance: " << _clustered_corrs[i].size () << std ::endl;

		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = _rototranslations[i].block<3,3>(0, 0);
		Eigen::Vector3f translation = _rototranslations[i].block<3,1>(0, 3);

		printf ("\n");
		printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
		printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
		printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
		printf ("\n");
		printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
	}
}


#endif
