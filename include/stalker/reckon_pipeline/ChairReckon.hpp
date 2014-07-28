#ifndef CHAIRRECON_BASE_H
#define CHAIRRECON_BASE_H

//PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>

#include "Shape3DGlobal.hpp"
#include "SegmentAndClustering.hpp"


template <typename T, typename DescriptorTypes>
class ChairRecon : public SegmentAndClustering<T, DescriptorTypes> {
public:
	
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > _rototranslations_chair;
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > _rototranslations_clustering;
	std::vector<typename pcl::PointCloud<T> > _clusters_chair;
	
	  ChairRecon(ShapeGlobal<T, DescriptorTypes>* object, ShapeGlobal<T, DescriptorTypes>* scene) : SegmentAndClustering<T, DescriptorTypes>(object, scene) {};

	  virtual void doPipeline();
	  virtual bool clustering(typename pcl::PointCloud<T>::Ptr cloud);
	  
	  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& getRoto() {
        return _rototranslations_chair;
    }
    
    virtual bool foundObject(){return ( _rototranslations_chair.size() != 0);}

	  

};

template <typename T, typename DescriptorTypes>
inline void ChairRecon<T, DescriptorTypes>::doPipeline()
{
	SegmentAndClustering<T, DescriptorTypes>::doPipeline();
	
	for(typename std::vector<typename pcl::PointCloud<T> >::iterator it=this->_clusters.begin();it!=this->_clusters.end();){
	
		//TODO Now is time to recognize a chair
		//Calcul min and max of the cloud

		std::cout << "working on. Totoal elements : "<<this->_clusters.size()<<std::endl;
		
		typename pcl::PointCloud<T>::Ptr shape(new pcl::PointCloud<T>((*it)));
		//shape=boost::shared_ptr<pcl::PointCloud<T> >((*it));
		
		
		stalker::voirPCL<T>(shape, shape);
		
		
		double y_max;
		double y_min;
		double x_min;
		double x_max;
		bool flag_cluster=false;
		float distance;
		
		typename pcl::PointCloud<T>::Ptr cloud_trimmed(new pcl::PointCloud<T>);
		std::string axe("y");
		y_min=stalker::minCloud<T>(shape, axe); //Y come from the camera frame convention Y is down. 
		y_max=stalker::maxCloud<T>(shape, axe);
		
		std::string axe2("x");
		x_max=stalker::minCloud<T>(shape, axe2); //X come from the camera frame convention Y is to the right. 
		x_min=stalker::maxCloud<T>(shape, axe2);
		
		double length=x_max-x_min;
		
		
		
		//Pass trhough to cut the top.
		//TODO maybe find a better value.
		double cut_at=(y_max+y_min)/2;
		std::cout<<"trimming "<< cut_at << " "<< y_max<<std::endl;
		stalker::passThrough<T>(shape, cloud_trimmed, "y", cut_at, y_max);
		
		
		
		
		//Clustering/Distance calculation
		int ii=0;
		while(flag_cluster==false && ii<15 ){
			flag_cluster=clustering(cloud_trimmed);
			cut_at=(cut_at+y_min)/2;
			ii++;
			std::cout<<"in"<<std::endl;
		}
		
		std::cout << "working on trimmed pcl "<<std::endl;
		/*pcl::copyPointCloud(cloud_trimmed, vec, *shape);
		stalker::voirPCL<T>(shape, shape);*/
		stalker::voirPCL<T>(cloud_trimmed, cloud_trimmed);
		
		// Calculate Point
		if(flag_cluster==false || ii>=15){
			//We did not find any chair in the end... :(
			std::cout<<"did not found any chair"<<std::endl;
		}
		else{
			//We see 3 or four legs
			if(_rototranslations_clustering.size()>2){
				
				//Calcul du barycentre !!!
				/*T point1;
				point1.x=_rototranslations_clustering[0](0,3);
				point1.y=_rototranslations_clustering[0](1,3);
				point1.z=_rototranslations_clustering[0](2,3);*/
				
				T barycentre;
				barycentre.x=0;
				barycentre.y=0;
				barycentre.z=0;
				int bob=0;
				
				for(typename std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >::iterator itt=this->_rototranslations_clustering.begin();bob<3;bob++){
					//Calculating barycenter
					
					std::cout <<"Calculating barycenter"<<std::endl;
					barycentre.x+=(*itt)(0,3);
					barycentre.y+=(*itt)(1,3);
					barycentre.z+=(*itt)(2,3);
					
					itt++;

				}
				
				if(_rototranslations_clustering.size()==3){
					
					//Need to add a point : First determine the diagonal !
					T point1;
					point1.x=_rototranslations_clustering[0](0,3);
					point1.y=_rototranslations_clustering[0](1,3);
					point1.z=_rototranslations_clustering[0](2,3);
					
					T point2;
					point2.x=_rototranslations_clustering[1](0,3);
					point2.y=_rototranslations_clustering[1](1,3);
					point2.z=_rototranslations_clustering[1](2,3);
					
					T point3;
					point3.x=_rototranslations_clustering[2](0,3);
					point3.y=_rototranslations_clustering[2](1,3);
					point3.z=_rototranslations_clustering[2](2,3);
					
					float dist12=pcl::squaredEuclideanDistance<T, T>(point1, point2);
					float dist23=pcl::squaredEuclideanDistance<T, T>(point2, point3);
					float dist31=pcl::squaredEuclideanDistance<T, T>(point3, point1);
					
					/*int diag=0;
					
					if(dist12>dist23){
						if(dist12>dist31){
							//Diagonal is 12
							diag=dist12;
						}
						else{
							//diagonal is 31
							diag=dist31;
						}
					}
					else{
						if(dist23>dist31){
							//diagonal is 23
							diag=dist23;
						}
						else{
							//diagonal is 31
							diag=dist31;
						}
					}*/
										
					//Test distance of point to point cloud || close to the centroid
					
					//Build the three possible points
					std::vector<T> list_of_possible_point;
					
					T p1;
					p1.x=(point2.x - point1.x) + (point3.x -point1.x) + point1.x; //Point opposed to point1
					p1.y=(point2.y - point1.y) + (point3.y -point1.y) + point1.y;
					p1.z=(point2.z - point1.z) + (point3.z -point1.z) + point1.z;
					list_of_possible_point.push_back(p1);
					
					T p2;
					p2.x=(point3.x - point2.x) + (point1.x -point2.x) + point2.x; //Point opposed to point2
					p2.y=(point3.y - point2.y) + (point1.y -point2.y) + point2.y;
					p2.z=(point3.z - point2.z) + (point1.z -point2.z) + point2.z;
					list_of_possible_point.push_back(p2);
					
					T p3;
					p3.x=(point2.x - point3.x) + (point1.x -point3.x) + point3.x; //Point opposed to point3
					p3.y=(point2.y - point3.y) + (point1.y -point3.y) + point3.y;
					p3.z=(point2.z - point3.z) + (point1.z -point3.z) + point3.z;
					list_of_possible_point.push_back(p3);
					 

					T pfinal;
					
					double o=0;
					double min=-1;
					int count=0;
					/*****************************************************************Use KDTree to calculate feetness***********************/
					
					for (typename std::vector<T>::iterator poi=list_of_possible_point.begin();poi!=list_of_possible_point.end();){
						srand (std::time (NULL));
						pcl::KdTreeFLANN<T> kdtree;
std::cout << "working on shape before kdtree "<<std::endl;
						stalker::voirPCL<T>(shape, shape);
						kdtree.setInputCloud (shape); //set the cloud as an input

						int K = 10;

						std::vector<int> pointIdxNKNSearch(K);
						std::vector<float> pointNKNSquaredDistance(K);

						std::cout << "K nearest neighbor search at (" << (*poi).x 
									<< " " << (*poi).y 
									<< " " << (*poi).z
									<< ") with K=" << K << std::endl;
						
						//Print the result
						if ( kdtree.nearestKSearch ( (*poi), K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
						{
	 						for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
							std::cout << "    "  <<   shape->points[ pointIdxNKNSearch[i] ].x 
										<< " " << shape->points[ pointIdxNKNSearch[i] ].y 
										<< " " << shape->points[ pointIdxNKNSearch[i] ].z 
										<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
								if(o==0){
									o=pointNKNSquaredDistance[i];
								}
								else{
									o+=pointNKNSquaredDistance[i];
								}
							}
						}
						if(o>min || min==-1){
							min=o;
							if(count==0){
								pfinal=p1;
							}
							if(count==1){
								pfinal=p2;
							}
							if(count==2){
								pfinal=p3;
							}
						}
						o=0;
						count++;
						poi++;
					}
					 
					//Second build the new point and add it
					
					barycentre.x+=pfinal.x;
					barycentre.y+=pfinal.y;
					barycentre.z+=pfinal.z;
					
					barycentre.x/=(_rototranslations_clustering.size()+1);
					barycentre.y/=(_rototranslations_clustering.size()+1);
					barycentre.z/=(_rototranslations_clustering.size()+1);
				}
				else{
					barycentre.x/=_rototranslations_clustering.size();
					barycentre.y/=_rototranslations_clustering.size();
					barycentre.z/=_rototranslations_clustering.size();
				}
				
				//Add the new pose
				Eigen::Matrix4f pose;
				pose<< 1,0,0,barycentre.x,
					0,1,0,barycentre.y,
					0,0,1,barycentre.z,
					0,0,0,1;
				
				_rototranslations_chair.push_back(pose);
				
				/*Calculating Point in between the clusters
				
				Point middle;
				middle.x=point1.x+ (0.5 *(point2.x -point1.x) ) ;
				middle.y=point1.y+ (0.5 *(point2.y -point1.y) ) ;
				middle.z=point1.z+ (0.5 *(point2.z -point1.z) ) ;*/
			}
			
			else{
				//If only two leg visible
				T barycentre;
				barycentre.x=0;
				barycentre.y=0;
				barycentre.z=0;
				
				for(typename std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >::iterator itt=this->_rototranslations_clustering.begin();itt!=this->_rototranslations_clustering.end();){
					//Calculating barycenter
					
					std::cout <<"Calculating barycenter only two leg"<<std::endl;
					barycentre.x+=(*itt)(0,3);
					barycentre.y+=(*itt)(0,3);
					barycentre.z+=(*itt)(0,3);
					
					itt++;

				}
				
				std::cout<<"NOT PROGRAMMED FOR NOW"<<std::endl;
				
				
				
			}
		
		}
		
		//TODO Know if it's the diagonal or not !
		
		
		//Fix a rotatranslation in rototranslations_chair
	
	it++;
	}
	
	std::cout<<"DONE"<<std::endl;
	
}

template <typename T, typename DescriptorTypes>
inline bool ChairRecon<T, DescriptorTypes>::clustering(typename pcl::PointCloud<T>::Ptr cloud)
{

    //plane segmentation using RANSAC
    pcl::SACSegmentation<T> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.segment(*inliers, *coefficients);

    //delete points on the plane
    pcl::ExtractIndices<T> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    typename pcl::PointCloud<T>::Ptr cloud_filtered(new pcl::PointCloud<T>);
    extract.filter(*cloud_filtered);

    //store the PointCloud using KdTree
    typename pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>);
    tree->setInputCloud(cloud_filtered);

    //object clustering
    pcl::EuclideanClusterExtraction<T> ece;
    ece.setClusterTolerance(0.02);
    ece.setMinClusterSize(100);
    ece.setMaxClusterSize(25000);
    ece.setSearchMethod(tree);
    ece.setInputCloud(cloud_filtered);
    std::vector<pcl::PointIndices> all_clusters_indices;
    ece.extract(all_clusters_indices);
	
	std::vector<typename pcl::PointCloud<T> > cluster_temp;

    //extract each cluster
	//Need at least 3 leg for now !
	if(all_clusters_indices.size()>2 && all_clusters_indices.size()<5){
		std::cout<<"we have NUMBER OF CLUSTER !!!! "<<all_clusters_indices.size()<<std::endl;
		pcl::PointCloud<T> cluster;
		for(unsigned int i = 0; i < all_clusters_indices.size(); ++i)
		{
			pcl::PointIndices::Ptr indices(new pcl::PointIndices);
			*indices = all_clusters_indices[i];
			extract.setInputCloud(cloud_filtered);
			extract.setIndices(indices);
			extract.setNegative(false);
			extract.filter(cluster);
			cluster_temp.push_back(cluster);
		}
		
		//compute the centroids of each cluster
		Eigen::Matrix< float, 4, 1 > trans;
		_rototranslations_clustering.empty();
		for(unsigned int i = 0; i < cluster_temp.size(); ++i)
		{
			pcl::compute3DCentroid(cluster_temp[i], trans);
			
			Eigen::Matrix4f pose;
			pose<< 1,0,0,trans[0],
				0,1,0,trans[1],
				0,0,1,trans[2],
				0,0,0,1;
			
			_rototranslations_clustering.push_back(pose);
		}
		
		return true;
	}
	else{
		return false;
	}
	
	
}


#endif