#ifndef PREPROCESSING_MALCOLM_H
#define PREPROCESSING_MALCOLM_H

#include <exception>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <deque>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d_omp.h>
#include <time.h>
#include "Postprocessing.hpp"


namespace stalker
{

	template<typename T>
	void removeNan(typename pcl::PointCloud<T>& cloud, typename pcl::PointCloud<T>& cloud_out);
	
	template<typename T>
	void removeNanNormals(typename pcl::PointCloud<T>& cloud, typename pcl::PointCloud<T>& cloud_out);
	
	template<typename T>
	void passThrough(typename pcl::PointCloud<T>::Ptr cloud, typename pcl::PointCloud<T>::Ptr cloud_out, const std::string& axis, double limits1, double limits2);
	
	template<typename T>
	void statisticalOutilerRemoval(typename pcl::PointCloud<T>::Ptr cloud, typename pcl::PointCloud<T>::Ptr cloud_out, int nneighbor, double thresh);
	
	template<typename T>
	void downSample(typename pcl::PointCloud<T>::Ptr cloud, typename pcl::PointCloud<T>::Ptr cloud_out, double radius);
	
	template <typename T, typename NormalType_template>
	void estimNormal(typename pcl::PointCloud<T>::Ptr cloud, typename pcl::PointCloud<NormalType_template>::Ptr cloud_out, double _k);
	
	template <typename T>
	double computeCloudResolution (typename pcl::PointCloud<T>::Ptr cloud);
	
	template <typename T>
	double computeCloudResolution (typename pcl::PointCloud<T>::Ptr cloud);
	
	template<typename T>
	bool gotnanTEST(const typename pcl::PointCloud<T>& cloud);
	
	template<typename T>
	bool gotinfTEST(const typename pcl::PointCloud<T>& cloud);
	
	template <typename T>
	double maxCloud(typename pcl::PointCloud<T>::Ptr cloud, std::string axis);
	
	template <typename T>
	double minCloud(typename pcl::PointCloud<T>::Ptr cloud, std::string axis);
	
	template<typename T>
	void calculateSizePCL(typename pcl::PointCloud<T>::Ptr cloud, double& x, double& y, double& z);
	
	template <typename T>
	void resizeCloud(typename pcl::PointCloud<T>::Ptr cloud, typename pcl::PointCloud<T>::Ptr cloud_out);
	
	template<typename T>
	void resizeCloudFactor(typename pcl::PointCloud<T>::Ptr cloud_in, typename pcl::PointCloud<T>::Ptr cloud_out, double factor);
	
	template<typename T>
	void center(typename pcl::PointCloud<T>::Ptr cloud, std::string axis);
	
	//Better but slower downsampling method
	template <typename T, typename NormalType_template>
	double downSampleUniformGrid(pcl::PointCloud<T>& cloud, pcl::PointCloud<NormalType_template>& cloud_normal, pcl::PointCloud<T>& cloud_out, double thresold);
	
	template<typename NormalType_template>
	double vCalculation(pcl::PointCloud<NormalType_template>& cloud_normal, size_t i, std::vector<int> index);
	
	double dotProduct(Eigen::Vector3d v, Eigen::Vector3d w);
	

	/*********************************************************************/
	
	
	
	struct timespec now;
	double tt_tic=0;

	double getTickCount(void) 
	{
	if (clock_gettime(CLOCK_MONOTONIC, &now))
	return 0;
	return now.tv_sec * 1000.0 + now.tv_nsec / 1000000.0;
	}
	
	void tic(){
		if (clock_gettime(CLOCK_MONOTONIC, &now))
			tt_tic= -1;
		tt_tic= now.tv_sec * 1000.0 + now.tv_nsec / 1000000.0;
	}
	void toc(){
		if(tt_tic==-1)
			std::cout<<"Clock error"<<std::endl;
		double tt_toc = getTickCount() - tt_tic;
		printf ("calculated in : %4.3f ms\n", tt_toc);
	}

	//**********************Actions*********************/
	template<typename T>
	void removeNan(typename pcl::PointCloud<T>& cloud, typename pcl::PointCloud<T>& cloud_out){
		std::vector<int> indices;
		//THIS DOES NOT COMPULE WITH is_dense set at true ! Since it is set as true by ROS (Whyyyyyy) I have to change it manually...
		pcl::removeNaNFromPointCloud(cloud,cloud_out, indices);
	}

	//BUGGED !
	template<typename T>
	void removeNanNormals(typename pcl::PointCloud<T>& cloud, typename pcl::PointCloud<T>& cloud_out){
		std::vector<int> indices;
		//pcl::removeNaNNormalsFromPointCloud(*cloud,*cloud_out, indices);
	}

	template<typename T>
	void passThrough(typename pcl::PointCloud<T>::Ptr cloud, typename pcl::PointCloud<T>::Ptr cloud_out, const std::string& axis, double limits1, double limits2){
		pcl::PassThrough<T> pass_x;
		pass_x.setInputCloud (cloud);
		pass_x.setFilterFieldName (axis);
		pass_x.setFilterLimits (limits1, limits2);	
		//pass_x.setFilterLimitsNegative (true);
		pass_x.filter (*cloud_out);
	}


	/*Outlier removal**/
	template<typename T>
	void statisticalOutilerRemoval(typename pcl::PointCloud<T>::Ptr cloud, typename pcl::PointCloud<T>::Ptr cloud_out, int nneighbor, double thresh){
		pcl::StatisticalOutlierRemoval<T> sor;
		sor.setInputCloud (cloud);
		sor.setMeanK (nneighbor);
		sor.setStddevMulThresh (thresh);
		sor.filter (*cloud_out);
	}

	/*DownSample*/
	template<typename T>
	void downSample(typename pcl::PointCloud<T>::Ptr cloud, typename pcl::PointCloud<T>::Ptr cloud_out, double radius){
		
		//pcl::PointCloud<T> sampled_indices;

		pcl::UniformSampling<T> uniform_sampling;
		uniform_sampling.setInputCloud (cloud);
		uniform_sampling.setRadiusSearch (radius);
		uniform_sampling.filter (*cloud_out);
		//pcl::copyPointCloud (*cloud, sampled_indices.points, *cloud_out);
		
		std::cout << "Shape total points: " << cloud->size () << "; Selected Keypoints: " << cloud_out->size () << std::endl;
	}
	
	
	template <typename T, typename NormalType_template>
	void estimNormal(typename pcl::PointCloud<T>::Ptr cloud, typename pcl::PointCloud<NormalType_template>::Ptr cloud_out, double _k){

		pcl::NormalEstimationOMP<T, NormalType_template> norm_est2;
		norm_est2.setKSearch (_k);
		norm_est2.setInputCloud (cloud);
		std::cout<<"compte the normals"<<std::endl;
		try{
			if(cloud->is_dense==true){
				norm_est2.compute (*(cloud_out));
			}
			else{
				throw std::invalid_argument("not dense");
			}
		}
		catch(std::exception const& e){
			std::cerr << "ERREUR SHAPE is not dense : " << e.what() << std::endl;
		}
	}

	/*Function calculating the resolution of a point cloud. The resolution is high when the concentration of point is small and small when the concentration of poitns is high.*/
	template <typename T>
	double computeCloudResolution (typename pcl::PointCloud<T>::Ptr cloud)
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

	//************************TESTS*********************/
	template<typename T>
	bool gotnanTEST(const typename pcl::PointCloud<T>& cloud){
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
	template<typename T>
	bool gotinfTEST(const typename pcl::PointCloud<T>& cloud){
		std::cout<<"testing infs"<<std::endl;
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

	//TODO
	template<typename T>
	double minCloud(typename pcl::PointCloud<T>::Ptr cloud, std::string axis){
		double min = std::numeric_limits<double>::quiet_NaN();
		if (cloud->isOrganized()){
			std::cout<<"Cloud organized"<<std::endl;
			for (int h=0; h<cloud->height; h++) {
				for (int w=0; w<cloud->width; w++){
					if(axis.compare("x")==0){ 
						if(isnan(min)){
							min=cloud->at(w, h).x;
						}
						else{
							if(min>cloud->at(w, h).x){
								min=cloud->at(w, h).x;
							}
						}			
					}
					if(axis.compare("y")==0){
						if(isnan(min)){
							min=cloud->at(w, h).y;
						}
						else{
							if(min>cloud->at(w, h).y){
								min=cloud->at(w, h).y;
							}
						}					
					}
					if(axis.compare("z")==0){
						if(isnan(min)){
							min=cloud->at(w, h).z;
						}
						else{
							if(min>cloud->at(w, h).z){
								min=cloud->at(w, h).z;
							}
						}					
					}
				}
			}
		}
		else{
			std::cout<<"Cloud not organized "<<axis.compare("x")<<" "<<cloud->width<<std::endl;
			for (int h=0; h<cloud->width; h++) {
				if(axis.compare("x")==0){ 
					//std::cout<<"checking x"<<std::endl;
					if(isnan(min)){
						min=cloud->points[h].x;
					}
					else{
						if(min>cloud->points[h].x){
							min=cloud->points[h].x;
						}
					}			
				}
				if(axis.compare("y")==0){
					if(isnan(min)){
						min=cloud->points[h].y;
					}
					else{
						if(min>cloud->points[h].y){
							min=cloud->points[h].y;
						}
					}					
				}
				if(axis.compare("z")==0){
					if(isnan(min)){
						min=cloud->points[h].z;
					}
					else{
						if(min>cloud->points[h].z){
							min=cloud->points[h].z;
						}
					}					
				}
			}
		}
		try{
			if(isnan(min)){
				throw std::invalid_argument("no min value found");
			}
		}
		catch(std::exception const& e){
			std::cerr << "Error : " << e.what() << std::endl;
		}
		
		return min;
	}	
	
	
	
	
	
	
	template<typename T>
	double maxCloud(typename pcl::PointCloud<T>::Ptr cloud, std::string axis){
		double max = std::numeric_limits<double>::quiet_NaN();
		if (cloud->isOrganized()){
			std::cout<<"Cloud organized"<<std::endl;
			for (int h=0; h<cloud->height; h++) {
				for (int w=0; w<cloud->width; w++){
					if(axis.compare("x")==0){ 
						if(isnan(max)){
							max=cloud->at(w, h).x;
						}
						else{
							if(max<cloud->at(w, h).x){
								max=cloud->at(w, h).x;
							}
						}			
					}
					if(axis.compare("y")==0){
						if(isnan(max)){
							max=cloud->at(w, h).y;
						}
						else{
							if(max<cloud->at(w, h).y){
								max=cloud->at(w, h).y;
							}
						}					
					}
					if(axis.compare("z")==0){
						if(isnan(max)){
							max=cloud->at(w, h).z;
						}
						else{
							if(max<cloud->at(w, h).z){
								max=cloud->at(w, h).z;
							}
						}					
					}
				}
			}
		}
		else{
			std::cout<<"Cloud not organized "<<axis.compare("x")<<" "<<cloud->width<<std::endl;
			for (int h=0; h<cloud->width; h++) {
				if(axis.compare("x")==0){ 
					std::cout<<"checking x"<<std::endl;
					if(isnan(max)){
						max=cloud->points[h].x;
					}
					else{
						if(max<cloud->points[h].x){
							max=cloud->points[h].x;
						}
					}			
				}
				if(axis.compare("y")==0){
					if(isnan(max)){
						max=cloud->points[h].y;
					}
					else{
						if(max<cloud->points[h].y){
							max=cloud->points[h].y;
						}
					}					
				}
				if(axis.compare("z")==0){
					if(isnan(max)){
						max=cloud->points[h].z;
					}
					else{
						if(max<cloud->points[h].z){
							max=cloud->points[h].z;
						}
					}					
				}
			}
		}
		try{
			if(isnan(max)){
				throw std::invalid_argument("no max value found");
			}
		}
		catch(std::exception const& e){
			std::cerr << "Error : " << e.what() << std::endl;
		}
		
		return max;
	}	

	
	
	
	
	
	template<typename T>
	void calculateSizePCL(typename pcl::PointCloud<T>::Ptr cloud, double& x, double& y, double& z){
	
		double min_x=minCloud<T>(cloud, "x");
		double max_x=maxCloud<T>(cloud, "x");
		
		double min_y=minCloud<T>(cloud, "y");
		double max_y=maxCloud<T>(cloud, "y");
		
		double min_z=minCloud<T>(cloud, "z");
		double max_z=maxCloud<T>(cloud, "z");
		
		x=max_x-min_x;
		y=max_y-min_y;
		z=max_z-min_z;
		
	}
	
	template <typename T>
	void resizeCloud(typename pcl::PointCloud<T>::Ptr cloud, typename pcl::PointCloud<T>::Ptr cloud_out){
		
		double in_x=0;
		double in_y=0;
		double in_z=0;
		
		double out_x=0;
		double out_y=0;
		double out_z=0;
		
		calculateSizePCL<T>(cloud, in_x, in_y, in_z);
		calculateSizePCL<T>(cloud_out, out_x, out_y, out_z);
		
		double factor_x=in_x/out_x;
		double factor_y=in_y/out_y;
		//double factor_z=in_z/out_z; <- background often visible, make the scaling be totally wrong !!!
		
		//double factor=(factor_x+factor_y+factor_z)/3;
		double factor=(factor_x+factor_y)/2;
		
		resizeCloudFactor<T>(cloud_out, cloud_out, factor);
		
// 		//stalker::voirPCL<T>(cloud_out, cloud);

	}
	
	
	
	template<typename T>
	void resizeCloudFactor(typename pcl::PointCloud<T>::Ptr cloud_in, typename pcl::PointCloud<T>::Ptr cloud_out, double factor){
		try{
			if(factor<0){
				throw std::invalid_argument("Factor < 0");
			}
		}
		catch(std::exception const& e){
			std::cerr << "Erreur in resize  : " << e.what() << std::endl;
			factor=1;
		}
		
		pcl::copyPointCloud(*cloud_in, *cloud_out);
			
		if (cloud_out->isOrganized()) {
			for (int h=0; h<cloud_out->height; h++) {
				for (int w=0; w<cloud_out->width; w++) {
					cloud_out->at(w,h).x = cloud_out->at(w, h).x*factor;
					cloud_out->at(w,h).y = cloud_out->at(w, h).y*factor;
					cloud_out->at(w,h).z = cloud_out->at(w, h).z*factor;
				}
			}
		}
		else{
			std::cerr << "Cloud not organized, can't apply the function : " <<cloud_in->width<<" "<<cloud_in->height<<" "<<cloud_out->width<<" "<<cloud_out->height<< std::endl;
			for (int h=0; h<cloud_out->width; h++) {
				cloud_out->points[h].x = cloud_out->points[h].x*factor;
				cloud_out->points[h].y = cloud_out->points[h].y*factor;
				cloud_out->points[h].z = cloud_out->points[h].z*factor;
			}
		}
		std::cout<<"Resied"<<std::endl;
		//stalker::voirPCL<T>(cloud_in, cloud_out);
	}
	
	template<typename T>
	void center(typename pcl::PointCloud<T>::Ptr cloud, std::string axis){
		double min=minCloud<T>(cloud, axis);
		for (int h=0; h<cloud->size(); h++){
			if(axis.compare("x")==0){ 
				cloud->points[h].x=cloud->points[h].x-min;	
			}
			if(axis.compare("y")==0){
				cloud->points[h].y=cloud->points[h].y-min;	
			}
			if(axis.compare("z")==0){
				cloud->points[h].z=cloud->points[h].z-min;	
			}
		}
	}
	
	
	/********************More downsample method*******************/
	template <typename T, typename NormalType_template>
	double downSampleUniformGrid(pcl::PointCloud<T>& cloud, pcl::PointCloud<NormalType_template>& cloud_normal, pcl::PointCloud<T>& cloud_out, double thresold){
		//Calculate the absolute value of the dot product of all normal with on point and divide by the number of point.
		std::cout<<"Downsample "<<thresold<<std::endl;
		try{
			if(cloud_normal.size()==cloud.size()){
				for (size_t i=0;i<cloud_normal.size();++i){
					std::cout<<"GO"<<std::endl;
					//************* Nearest Neighbor search ************//
					typename pcl::PointCloud<NormalType_template>::Ptr p(new pcl::PointCloud<NormalType_template>);
					*p=cloud_normal;
					pcl::KdTreeFLANN<NormalType_template> kdtree;
					kdtree.setInputCloud (p);
					NormalType_template searchPoint;	 
					int K = 10;
					std::vector<int> pointIdxNKNSearch;
					std::vector<float> pointNKNSquaredDistance;
					if(kdtree.radiusSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance)>0){
						//************** V CALCULATION *********************//
						//cloud_out.points[i].strength=vCalculation(cloud_normal, i, pointIdxNKNSearch);
						if(vCalculation(cloud_normal, i, pointIdxNKNSearch) >thresold){

							cloud_out.push_back(T(cloud.points[i].x,cloud.points[i].y,cloud.points[i].z));
							
						}
							
					}
					else{
						cloud_out.push_back(T(cloud.points[i].x,cloud.points[i].y,cloud.points[i].z));
					}
					
				}
				
				std::cout <<"Size before " << cloud_normal.size()<< " after "<< cloud_out.size() <<std::endl;
				//**** If V under the thresold then the point is sampled. The smaller V, the flatter**/
			}
			else{
				throw std::invalid_argument("Cloud and normal don't have the same size for the uniform grib downsampling : ");//Need to figure out how to change and know the shape's names !! Maybe a yaml file...
			}
		}
		catch(std::exception const& e){
			std::cerr << "Erreur : " << e.what() << cloud_normal.size()<<"!="<<cloud.size()<<std::endl;
			exit(0);
		}
		
		
	}

	template<typename NormalType_template>
	double vCalculation(pcl::PointCloud<NormalType_template>& cloud_normal, size_t i, std::vector<int> index){
		//V=absolute value of dot product / nd of points
		
		double sum;
		for (size_t j = 0; j < index.size (); ++j){
			sum=sum+dotProduct(	Eigen::Vector3d(cloud_normal.points[i].normal_x,cloud_normal.points[i].normal_y,cloud_normal.points[i].normal_z), 	   Eigen::Vector3d(cloud_normal.points[index[j]].normal_x,cloud_normal.points[index[j]].normal_y,cloud_normal.points[index[j]].normal_z));
		}
		sum=sum/index.size();
		sum=1-sum;
		return sum;
	}


	double dotProduct(Eigen::Vector3d v, Eigen::Vector3d w){
		double dot=v.dot(w);
		if( dot<0){
			dot=-dot;
		}
		return dot;
	}
	
	

	
	
}


#endif
