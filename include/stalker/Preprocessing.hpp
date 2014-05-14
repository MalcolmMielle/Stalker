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
		pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);
	}
	
	//BUGGED !
	void removeNanNormals(typename pcl::PointCloud<T>::Ptr& cloud){
		std::vector<int> indices;
		//pcl::removeNaNNormalsFromPointCloud(*cloud,*cloud, indices);
	}
	
	bool gotnanTEST(typename pcl::PointCloud<T>::Ptr& cloud){	
		for(int x=0;x<cloud->width;++x){
			for(int y=0;y<cloud->height;++y){
				T point(cloud->at(x,y));
				if(isnan(point.x) || isnan(point.y) || isnan(point.z)){
					return true;
				}
			}
		}
		return false;
	}
	
	bool gotinfTEST(typename pcl::PointCloud<T>::Ptr& cloud){
		for(int x=0;x<cloud->width;++x){
			for(int y=0;y<cloud->height;++y){
				T point(cloud->at(x,y));
				if(isinf(point.x) || isinf(point.y) || isinf(point.z)){
					return true;
				}
			}
		}
		return false;
	}
};

#endif