#ifndef CORRESGROUP_RECKON_H
#define CORRESGROUP_RECKON_H

#include "CorrespGroupingBase.hpp"

#include <pcl/visualization/pcl_visualizer.h>

/*The behavior you experienced is because when PCL transforms a point cloud into a matrix data structure used by FLANN, does so with the help of a PointRepresentation class and by default this class uses only the first 3 floats of each point unless a specialization for the new data type is provided.

You can get the expected behavior for your code by adding the following specialization for DefaultPointRepresentation:
*/

namespace pcl {
template <>
class DefaultPointRepresentation<pcl::Histogram<153> > : public PointRepresentation<pcl::Histogram<153> >
{
public:
  DefaultPointRepresentation ()
  {
    nr_dimensions_ = 153;
  }

  virtual void
  copyToFloatArray (const pcl::Histogram<153>  &p, float * out) const
  {
    for (int i = 0; i < nr_dimensions_; ++i)
      out[i] = p.histogram[i];
  }
};
}


/******************************************************/
/*********************DEFAULT BASE*********************/
/******************************************************/

template <typename T, typename DescriptorTypes>
class CorrespGrouping : public CorrespGroupingBaseBase<T, DescriptorTypes> {
	protected: 

	public : 
		
	CorrespGrouping(ShapeLocal<T, DescriptorTypes>* object, ShapeLocal<T, DescriptorTypes>* scene) : CorrespGroupingBaseBase<T, DescriptorTypes>(object, scene) {
		this->useHough();
		
	};
	
	virtual void point2PointCorrespondance();
	

};


/*****FUNCTION IS OK ERROR COME FROM BEFORE ********/
template <typename T, typename DescriptorTypes>
inline void CorrespGrouping<T, DescriptorTypes>::point2PointCorrespondance(){
	
	stalker::tic();
	
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
		if(kop==10){std::cout<<"Calculating the Point to Point correspondence..."<<std::endl;kop=0;}
		//  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
		if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) {
			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			this->_model_scene_corrs->push_back (corr);
		}
		kop++;
		
	}
	std::cout<<std::endl<<"Point 2 point correspondance grouping ";
	stalker::toc();
	std::cout<<"number of entries "<<kop<<std::endl;
	std::cout << "Correspondences found: " << this->_model_scene_corrs->size() << std::endl;
}



/******************************************************/
/***********************SPIN IMAGE*********************/
/******************************************************/

/*Sin image
 * Not scale invariant !
 * 
 */

template <typename T>
class CorrespGrouping<T, pcl::Histogram<153> > : public CorrespGroupingBaseBase<T, pcl::Histogram<153> > {
	protected: 
	
	public : 
	CorrespGrouping(ShapeLocal<T, pcl::Histogram<153> >* object, ShapeLocal<T, pcl::Histogram<153> >* scene) : CorrespGroupingBaseBase<T, pcl::Histogram<153> >(object, scene) {
		this->useGeometricConsistency();
	};
	
	virtual void point2PointCorrespondance();
	
};


/*****FUNCTION IS OK ERROR COME FROM BEFORE ********/
template <typename T>
inline void CorrespGrouping<T, pcl::Histogram<153> >::point2PointCorrespondance(){
		
	//declare variables for Kd tree search
	
	 //TODO change that for a better Kd tree search for high dimension. I.E = KDTreeIndex !
	 
	 //  pcl::KdTreeFLANN wraps flann::KDTreeSingleIndex which is optimized for low dimensional exact search and is not ideal for 153 dimensional descriptors. For high dimensional points you should be using an approximate nearest neighbor index from FLANN, such as flann::KDTreeIndex or flann::KMeansIndex
	 
	pcl::KdTreeFLANN<pcl::Histogram<153> > match_search;
	
	int match = 0;
	int temp_dist2NN;
	pcl::Histogram<153> searchFeature;

	//search for 2NN and check for ratio threshold
	int K = 2;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

	//initilization for kd-tree
	match_search.setInputCloud (this->_object->getDescr());

	//compare each query point with each scene point
	int numFoundMatches;
	for (size_t c = 0; c < this->_scene->getDescr()->size(); c++)
	{
		searchFeature = this->_scene->getDescr()->points.at(c);

		if(std::isnan(searchFeature.histogram[0])){
			continue;
		}

		numFoundMatches = match_search.nearestKSearch(searchFeature,K,pointIdxNKNSearch,pointNKNSquaredDistance);

		if(numFoundMatches>0){
			temp_dist2NN = pointNKNSquaredDistance.at(0);

		}
		if (temp_dist2NN<0.25f){
			match++;
			pcl::Correspondence corr (pointIdxNKNSearch[0], static_cast<int> (c), pointNKNSquaredDistance[0]);
			this->_model_scene_corrs->push_back (corr);
		}

		std::cout<<"point " <<c<<" in query matches point "<<pointIdxNKNSearch.at(0)<<" in database."<<std::endl;
	}
	//show status
	std::cout<<"total match:"<<match<<std::endl;

}






#endif
