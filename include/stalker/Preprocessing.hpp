#ifndef PREPROCESSING_MALCOLM_H
#define PREPROCESSING_MALCOLM_H

#include <exception>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <deque>
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
	
	void removeNanNormals(typename pcl::PointCloud<T>::Ptr& cloud){
		std::vector<int> indices;
		pcl::removeNaNNormalsFromPointCloud(*cloud,*cloud, indices);
	}
	
	bool nanTEST(typename pcl::PointCloud<T>::Ptr& cloud){	
		for(int x=0;x<cloud->width;++x){
			for(int y=0;y<cloud->height;++y){
				pcl::T point(cloud->at(x,y));
				if(!isnan(point->x) || !isnan(point.y) || !isnan(point.z)){
					return false
				}
			}
		}
		return true
	}
	
	bool infTEST(typename pcl::PointCloud<T>::Ptr& cloud){
		for(int x=0;x<cloud->width;++x){
			for(int y=0;y<cloud->height;++y){
				pcl::T point(cloud->at(x,y));
				if(!isinf(point->x) || !isinf(point.y) || !isinf(point.z)){
					return false
				}
			}
		}
		return true
	}
};

#endif