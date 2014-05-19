#ifndef PREPROCESSING_MALCOLM_H
#define PREPROCESSING_MALCOLM_H

#include <exception>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <deque>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

template<typename T>
class Preprocessing{
	protected : 
	
	public : 
	Preprocessing(){};
	
	void removeNan(typename pcl::PointCloud<T>::Ptr& cloud){
		std::vector<int> indices;
		//THIS DOES NOT COMPULE WITH is_dense set at true ! Since it is set as true by ROS (Whyyyyyy) I have to change it manually...
		pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);
	}
	
	//BUGGED !
	void removeNanNormals(typename pcl::PointCloud<T>::Ptr& cloud){
		std::vector<int> indices;
		//pcl::removeNaNNormalsFromPointCloud(*cloud,*cloud, indices);
	}
	
	bool gotnanTEST(typename pcl::PointCloud<T>::Ptr& cloudy){
		std::cout<<"testing nans"<<std::endl;
		pcl::PointCloud<T> cloud=*cloudy;
		if (cloud.isOrganized ()){
			for(int x=0;x<cloud.width;++x){
				for(int y=0;y<cloud.height;++y){
					T point(cloud[y+(x*cloud.width)]);
					if(isnan(point.x) || isnan(point.y) || isnan(point.z)){
						std::cout<<"ERROR AT "<<x<<" "<<y<<" WITH "<<point.x<<" "<<point.y<<" "<<point.z<<std::endl;
						return true;
					}
				}
			}
			return false;
		}
		else{
			for(int x=0;x<cloud.width;++x){
				T point(cloud[x]);
				if(isnan(point.x) || isnan(point.y) || isnan(point.z)){
					std::cout<<"ERROR AT "<<x<<" WITH "<<point.x<<" "<<point.y<<" "<<point.z<<std::endl;
					return true;
				}
			}
			return false;	
		}
	}
	
	bool gotinfTEST(typename pcl::PointCloud<T>::Ptr& cloudy){
		std::cout<<"testing infs"<<std::endl;
		pcl::PointCloud<T> cloud=*cloudy;
		if (cloud.isOrganized ()){
			for(int x=0;x<cloud.width;++x){
				for(int y=0;y<cloud.height;++y){
					T point(cloud[y+(x*cloud.width)]);
					if(isinf(point.x) || isinf(point.y) || isinf(point.z)){
						return true;
					}
				}
			}
			return false;
		}
		else{
			for(int x=0;x<cloud.width;++x){
				T point(cloud[x]);
				if(isnan(point.x) || isnan(point.y) || isnan(point.z)){
					std::cout<<"ERROR AT "<<x<<" WITH "<<point.x<<" "<<point.y<<" "<<point.z<<std::endl;
					return true;
				}
			}
			return false;	
		}
		
	}
};

#endif