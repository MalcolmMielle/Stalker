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
#include <Preprocessing.hpp>
#include "Postprocessing.hpp"
#include <pcl/registration/ia_ransac.h>

/******************Base de la base*******************/
//Used to have all the parameters used somewhere before.
//The last Corresp Grouping is merely used to be able to call them all by the same name.


template <typename T, typename DescriptorTypes>
class CorrespGroupingBaseBase : public Pipeline<T, DescriptorTypes> {
	protected: 
	//ShapeLocal<T, pcl::SHOT352>* _object; //Don't need because it take a Shape* from main.
	//ShapeLocal<T, pcl::SHOT352>* _scene; // Need to be initialise because it takes a Cloud in argument.
	bool resol;
	
	double _rf_rad; //Referance frame radius given by the user. default 0.015
	double _rf_rad_effective; //Referance frame radius used in the calculation. Useful in case we want to use resolution invariance.
	double _cg_size; //Cluster size given by the user. default 0.01
	double _cg_size_effective; //Cluster size used in the calculation. Useful in case we want to use resolution invariance.
	double _cg_thresh; //Cluster thressold default 5
	
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > _rototranslations; //Eigen vectors. give the three vector defining the position. The smallest one is the z.
	std::vector<pcl::Correspondences> _clustered_corrs;
	pcl::CorrespondencesPtr _model_scene_corrs ;
	
	bool _useGeometricConsistency;
	bool _useHough;	
	double _postprocessing_icp_fitness_thresh;
	
	public : 
		
	CorrespGroupingBaseBase(ShapeLocal<T, DescriptorTypes>* object, ShapeLocal<T, DescriptorTypes>* scene) : Pipeline<T, DescriptorTypes>(object, scene),/* _object(object), _scene(scene),*/resol(false), _rf_rad(0.015), _rf_rad_effective(0.015), _cg_size(0.01), _cg_size_effective(0.01), _cg_thresh(5.0), _model_scene_corrs(new pcl::Correspondences ()), _useGeometricConsistency(true), _useHough(false),_postprocessing_icp_fitness_thresh(6e-5) {};
	
	virtual void doPipeline();
	virtual void doPipeline(const sensor_msgs::PointCloud2ConstPtr& cloudy);
	//virtual void doPipelineOld();
	
	//new stuff
	virtual void setResol(bool y){resol=y;}
	virtual void setFrameRadius(double rf){_rf_rad=rf;}
	virtual void setClusterSize(double rf){_cg_size=rf;}
	virtual void setFrameRadiusEffective(double rf){_rf_rad_effective=rf;}
	virtual void setClusterSizeEffective(double rf){_cg_size_effective=rf;}
	virtual void setClusterThresold(double rf){_cg_thresh=rf;}
	virtual void setPostProcICPThresh(double i){_postprocessing_icp_fitness_thresh=i;}

	virtual void useHough(){_useHough=true; _useGeometricConsistency=false;}
	virtual void useGeometricConsistency(){_useHough=false; _useGeometricConsistency=true;}
	
	virtual double getFrameRadius(){return _rf_rad;}
	virtual double getClusterSize(){return _cg_size;}
	virtual double getFrameRadiusEffective(){return _rf_rad_effective;}
	virtual double getClusterSizeEffective(){return _cg_size_effective;}
	virtual double getClusterThresold(){return _cg_thresh;}
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& getRoto(){return _rototranslations;}
	std::vector<pcl::Correspondences>& getClust(){return _clustered_corrs;}
	pcl::CorrespondencesPtr& getModelCorres(){return _model_scene_corrs;}
	
	
	virtual void resolutionInvariance();
	virtual void clusteringHough();
	virtual void point2PointCorrespondance()=0;
	virtual void postProcessing();
	//virtual void estimNormal();
	
	virtual void affiche();
	virtual void printinfo(){
		std::cout<<"INSIDE THE CLASS"<<std::endl;
		std::cout<<"heig "<<this->_scene->getCloud()->height<<" width "<<this->_scene->getCloud()->width<<" size "<< this->_scene->getCloud()->size()<<" dense "<< this->_scene->getCloud()->is_dense<< " Organized "<<this->_scene->getCloud()->isOrganized() <<std::endl;
		
		std::cout<<"INSIDE THE CLASS model"<<std::endl;
		std::cout<<"heig "<<this->_object->getCloud()->height<<" width "<<this->_object->getCloud()->width<<" size "<< this->_object->getCloud()->size()<<" dense "<< this->_object->getCloud()->is_dense<<" Organized "<<this->_object->getCloud()->isOrganized() <<std::endl;
		
		std::cout<<"The correspondance grouping : "<<" resolution " <<resol<< " reference frame radius " <<_rf_rad<< "reference radius effective "<<_rf_rad_effective<<" cluster size "<<_cg_size<<" ffective "<<_cg_size_effective<<" threshold clustering " << _cg_thresh<< " roto size "<<_rototranslations.size()<<" cluster size "<<_clustered_corrs.size()<< "model correspon size "<<_model_scene_corrs->size()<<std::endl;
	
	}
	
	virtual void eraseRot();
	virtual void eraseClust();
	virtual void eraseCorres();
	
	//OVERWRITTEN FUNTION
	virtual void print(Gui<T,DescriptorTypes>* g){
		std::cout << "Hello, I'm a CG =D" << std::endl;
		g->printPipeline((*this));
	}
	
	//OVERWRITTEN FUNTION
	//virtual void setScene(typename pcl::PointCloud<T>::Ptr& obj){this->_scene->set(obj);}
	//virtual void setObject(typename pcl::PointCloud<T>::Ptr& obj){this->_object->set(obj);}

};


template <typename T, typename DescriptorTypes>
inline void CorrespGroupingBaseBase<T, DescriptorTypes>::doPipeline()
{
	if(this->_scene->getCloud()->height>0 && this->_scene->getCloud()->width>0 && this->_object->getCloud()->height>0 && this->_object->getCloud()->height>0 ){
		std::cout<<"ENTER THE PIPELINE ************************************"<<std::endl;
		std::cout<<"erasing"<<std::endl;
		
		this->eraseClust();
		this->eraseRot();
		this->eraseCorres();
		
		printinfo();
		
		if(resol==true){
			std::cout << "Rosulation"<<std::endl;
			resolutionInvariance();
		}
		this->point2PointCorrespondance();
		clusteringHough();
	}
}

template <typename T, typename DescriptorTypes>
inline void CorrespGroupingBaseBase<T, DescriptorTypes>::doPipeline(const sensor_msgs::PointCloud2ConstPtr& cloudy){
	typename pcl::PointCloud<T>::Ptr p(new pcl::PointCloud<T>() );
	pcl::fromROSMsg(*cloudy, *p);
	this->_scene->update(p);	
}





template <typename T, typename DescriptorTypes>
inline void CorrespGroupingBaseBase<T, DescriptorTypes>::resolutionInvariance(){

//static_cast<float> (computeCloudResolution (this->_shape));
	//ATTENTION CHECK QUE CA A ETE FAIT CHEZ SHAPE !!!
	if (this->_scene->getResolutionState()==true && this->_object->getResolutionState()==true)
	{
		double resolution=this->_object->getResolution();
		//this->_shape_ss   *= resolution;
		this->_rf_rad_effective = _rf_rad*resolution;
		//this->_descrRad  *= resolution;
		this->_cg_size_effective = _rf_rad*resolution;
	}

}


template <typename T, typename DescriptorTypes>
inline void CorrespGroupingBaseBase<T, DescriptorTypes>::affiche()
{
	//TODO Correct that ! =)
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
	std::cout << "end of affiche: " << _rototranslations.size () << std::endl;
}


template <typename T, typename DescriptorTypes>
inline void CorrespGroupingBaseBase<T, DescriptorTypes>::eraseRot(){
	for(typename std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >::iterator it=this->_rototranslations.begin();it!=this->_rototranslations.end();){
		this->_rototranslations.erase(it);
	}
}

template <typename T, typename DescriptorTypes>
inline void CorrespGroupingBaseBase<T, DescriptorTypes>::eraseClust(){
	for(typename std::vector<pcl::Correspondences>::iterator it=this->_clustered_corrs.begin();it!=this->_clustered_corrs.end();){
		this->_clustered_corrs.erase(it);
	}
}

template <typename T, typename DescriptorTypes>
inline void CorrespGroupingBaseBase<T, DescriptorTypes>::eraseCorres(){
	for(typename pcl::Correspondences::iterator it=this->_model_scene_corrs->begin();it!=this->_model_scene_corrs->end();){
		//delete(it);
		this->_model_scene_corrs->erase(it);
	}
}




template <typename T, typename DescriptorTypes>
inline void CorrespGroupingBaseBase<T, DescriptorTypes >::clusteringHough(){
	    //
	//  Compute (Keypoints) Reference Frames only for Hough
	
	pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
	pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

	//USED THAT IN PCL 1.7
	pcl::BOARDLocalReferenceFrameEstimation<T, NormalType, RFType> rf_est;
	rf_est.setFindHoles (true);
	std::cout <<"Defining the radius at " <<this->_rf_rad_effective<<std::endl; 
	rf_est.setRadiusSearch (this->_rf_rad_effective);

	rf_est.setInputCloud (this->_object->getKeypoints());
	rf_est.setInputNormals (this->_object->getNormals());
	rf_est.setSearchSurface (this->_object->getCloud());
	std::cout <<"Computing model"<<std::endl; 
	rf_est.compute (*model_rf);

	rf_est.setInputCloud (this->_scene->getKeypoints());
	rf_est.setInputNormals (this->_scene->getNormals());
	rf_est.setSearchSurface (this->_scene->getCloud());
	std::cout <<"Computing scene"<<std::endl; 
	rf_est.compute (*scene_rf); 

	//  Clustering
	if(_useHough==true){
		std::cout<<"Using Hough Clustering"<<std::endl;
		pcl::Hough3DGrouping<T, T, RFType, RFType> clusterer; //undefined reference in .o
		clusterer.setHoughBinSize (this->_cg_size_effective);
		clusterer.setHoughThreshold (this->_cg_thresh);
		clusterer.setUseInterpolation (true);
		clusterer.setUseDistanceWeight (false);

		clusterer.setInputCloud (this->_object->getKeypoints());
		clusterer.setInputRf (model_rf);
		clusterer.setSceneCloud (this->_scene->getKeypoints());
		clusterer.setSceneRf (scene_rf);
		clusterer.setModelSceneCorrespondences (this->_model_scene_corrs);

		//clusterer.cluster (clustered_corrs);
		clusterer.recognize (this->_rototranslations, this->_clustered_corrs);
	}

	if(_useGeometricConsistency==true){
		std::cout<<"Using Geometric Consistency"<<std::endl;
		pcl::GeometricConsistencyGrouping<T, T> gc_clusterer;
		gc_clusterer.setGCSize (this->_cg_size_effective);
		gc_clusterer.setGCThreshold (this->_cg_thresh);

		gc_clusterer.setInputCloud (this->_object->getKeypoints());
		gc_clusterer.setSceneCloud (this->_scene->getKeypoints());
		gc_clusterer.setModelSceneCorrespondences (this->_model_scene_corrs);

		//gc_clusterer.cluster (clustered_corrs);
		std::cout <<"Recognize scene"<<std::endl; 
		gc_clusterer.recognize (this->_rototranslations, this->_clustered_corrs);
		std::cout <<"Done"<<std::endl;
	}
	postProcessing();
}


template<typename T, typename DescriptorTypes>
inline void CorrespGroupingBaseBase<T, DescriptorTypes >::postProcessing(){
	std::cout<<"Post Processing"<<std::endl;

	//TODO TESTIN PART TO REMOVE
	double best=1;
	for(typename std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >::iterator it=this->_rototranslations.begin();it!=this->_rototranslations.end();)
	{
		std::cout<<"Post Processing"<<std::endl;
		//Eigen::Matrix4f transformation_matrix=(*it); <- (*it) is the transformation matrix
		
		typename pcl::PointCloud<T>::Ptr transformed_cloud (new pcl::PointCloud<T> ());	// A pointer on a new cloud
		pcl::transformPointCloud (*(this->_object->getCloud()), *transformed_cloud, (*it));
		
		int res=-1;
		double fitness=-1;

		Eigen::Matrix4f transformation_matrix_icp= Eigen::Matrix4f::Identity();
		
		stalker::ICPtransform<T>(transformed_cloud, this->_scene->getCloud(), res, fitness,transformation_matrix_icp);

		//TODO TO REMOVE / TEST
		if(best>fitness){
			best=fitness;
			//stalker::voirPCL<T>(transformed_cloud,this->_scene->getCloud());
			
			Eigen::Matrix4f transformation_matrix2=(*it);
			(transformation_matrix2)=(transformation_matrix2)*transformation_matrix_icp;
			
			typename pcl::PointCloud<T>::Ptr transformed_cloud_test (new pcl::PointCloud<T> ());
	
			pcl::transformPointCloud (*(this->_object->getCloud()), *transformed_cloud_test, transformation_matrix2);
			
			std::cout <<"NEW BEST! "<< best<< " cloud size model "<<this->_object->getCloud()->size()<<std::endl;
			//stalker::voirPCL<T>(transformed_cloud_test,this->_scene->getCloud());
		}
	
		
		//False positive !
		if(res==0 || fitness>_postprocessing_icp_fitness_thresh){
			//std::cout<<"Bad model ! "<<fitness<<std::endl;
			//delete(*it);
			this->_rototranslations.erase(it);
		}
		else{
			std::cout<<"Found one good instance !"<<std::endl;
			//Multiplying the matrix the have the total transformation !
			(*it)=(*it)*transformation_matrix_icp;
			//pcl::transformPointCloud (*(this->_object->getCloud()), *transformed_cloud, (*it));
			++it;	
		}
		
	}
}



#endif