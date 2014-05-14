#ifndef CORRESGROUP_RECKON_BASE_H
#define CORRESGROUP_RECKON_BASE_H

#include "Pipeline.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/correspondence.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/features/board.h>
#include <pcl/features/shot_lrf.h>
#include <pcl/recognition/cg/geometric_consistency.h>


template <typename T, typename DescriptorTypes>
class CorrespGroupingBase : public Pipeline<T, DescriptorTypes> {
	protected: 
	pcl::CorrespondencesPtr _model_scene_corrs ;
	
	public : 
		
	CorrespGroupingBase(ShapeLocal<T, DescriptorTypes>* object, ShapeLocal<T, DescriptorTypes>* scene) : Pipeline<T, DescriptorTypes>(object, scene), _model_scene_corrs(new pcl::Correspondences ()) {};
	
	virtual void point2PointCorrespondance();
};


/*****FUNCTION IS OK ERROR COME FROM BEFORE ********/
template <typename T, typename DescriptorTypes>
inline void CorrespGroupingBase<T, DescriptorTypes>::point2PointCorrespondance(){
	
	pcl::KdTreeFLANN<DescriptorTypes> match_search;
	
	match_search.setInputCloud (this->_object->getDescr());
	std::cout << "Scene descirptor size "<< this->_scene->getDescr()->size()<<" object descr size "<< this->_object->getDescr()->size()<< std::endl;
	int kop=0;
	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (size_t i = 0; i < this->_scene->getDescr()->size(); ++i){
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		//skipping NaNs
		if (!pcl_isfinite (this->_scene->getDescr()->at (i).descriptor[0])) {
			continue;
		}

		int found_neighs = match_search.nearestKSearch (this->_scene->getDescr()->at(i), 1, neigh_indices, neigh_sqr_dists);

		//  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
		if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) {
			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			_model_scene_corrs->push_back (corr);
			kop++;
		}
		
	}
	std::cout<<"number of entries "<<kop<<std::endl;
	std::cout << "Correspondences found: " << _model_scene_corrs->size() << std::endl;
}





/************************SPIN IMAGE************************/

template <typename T>
class CorrespGroupingBase<T, pcl::Histogram<153> > : public Pipeline<T, pcl::Histogram<153> > {
	protected: 
		
	pcl::CorrespondencesPtr _model_scene_corrs ;
	
	public : 
	CorrespGroupingBase(ShapeLocal<T, pcl::Histogram<153> >* object, ShapeLocal<T, pcl::Histogram<153> >* scene) : Pipeline<T, pcl::Histogram<153> >(object, scene), _model_scene_corrs(new pcl::Correspondences ()) {};
	
	virtual void point2PointCorrespondance();
};


/*****FUNCTION IS OK ERROR COME FROM BEFORE ********/
template <typename T>
inline void CorrespGroupingBase<T, pcl::Histogram<153> >::point2PointCorrespondance(){
	
	/*pcl::KdTreeFLANN<pcl::Histogram<153> > match_search;
	
	match_search.setInputCloud (this->_object->getDescr());
	std::cout << "Scene descirptor size "<< this->_scene->getDescr()->size()<<" object descr size "<< this->_object->getDescr()->size()<< std::endl;
	int kop=0;
	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (size_t i = 0; i < this->_scene->getDescr()->size(); ++i){
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		//skipping NaNs
		if (!pcl_isfinite (this->_scene->getDescr()->at (i).descriptor[0])) {
			continue;
		}

		int found_neighs = match_search.nearestKSearch (this->_scene->getDescr()->at(i), 1, neigh_indices, neigh_sqr_dists);

		//  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
		if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) {
			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			_model_scene_corrs->push_back (corr);
			kop++;
		}
		
	}
	std::cout<<"number of entries "<<kop<<std::endl;
	std::cout << "Correspondences found: " << _model_scene_corrs->size() << std::endl;*/
}

#endif