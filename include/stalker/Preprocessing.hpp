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
#include <pcl/keypoints/uniform_sampling.h>
#include <time.h>

namespace stalker
{
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
	void passThrough(typename pcl::PointCloud<T>::Ptr cloud, typename pcl::PointCloud<T>::Ptr cloud_out, const std::string& axis, int limits1, int limits2){
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
		
		pcl::PointCloud<int> sampled_indices;

		pcl::UniformSampling<T> uniform_sampling;
		uniform_sampling.setInputCloud (cloud);
		uniform_sampling.setRadiusSearch (radius);
		uniform_sampling.compute (sampled_indices);
		pcl::copyPointCloud (*cloud, sampled_indices.points, *cloud_out);
		
		std::cout << "Shape total points: " << cloud->size () << "; Selected Keypoints: " << cloud_out->size () << std::endl;
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

}

/*template<typename T>
class Preprocessing{
	protected : 
	typename pcl::PointCloud<T>::Ptr _input; //Last Scene receive
	typename pcl::PointCloud<T>::Ptr _output; //Last Object receive
	double _shape_ss; //shape_sampling size
	
	public : 
	Preprocessing() : _input(new pcl::PointCloud<T>()),_output(new pcl::PointCloud<T>()) {};
	
	//******µµµµAccesseur***********
	virtual typename pcl::PointCloud<T>::Ptr getInput(){return _input;}
	virtual typename pcl::PointCloud<T>::Ptr getOutput(){return _output;}
	virtual void setInput(typename pcl::PointCloud<T>::Ptr in){_input=in;}
	virtual void setOutput(typename pcl::PointCloud<T>::Ptr out){_output=out;}
	virtual void setSamplingSize(double r){_shape_ss=r;}
	virtual double getSamplingSize(){return _shape_ss;}
	
	//*********************Main functionù*****************
	virtual void doPreprocessing(typename pcl::PointCloud<T>& cloud, typename pcl::PointCloud<T>& cloud_out)=0;
	
	
	//**********************Actions*********************
	
	virtual void removeNan(typename pcl::PointCloud<T>& cloud, typename pcl::PointCloud<T>& cloud_out){
		std::vector<int> indices;
		//THIS DOES NOT COMPULE WITH is_dense set at true ! Since it is set as true by ROS (Whyyyyyy) I have to change it manually...
		pcl::removeNaNFromPointCloud(cloud,cloud_out, indices);
	}
	virtual void removeNan(){
		std::vector<int> indices;
		//THIS DOES NOT COMPULE WITH is_dense set at true ! Since it is set as true by ROS (Whyyyyyy) I have to change it manually...
		pcl::removeNaNFromPointCloud(*_input, *_output, indices);
	}
	
	//BUGGED !
	virtual void removeNanNormals(typename pcl::PointCloud<T>& cloud, typename pcl::PointCloud<T>& cloud_out){
		std::vector<int> indices;
		//pcl::removeNaNNormalsFromPointCloud(*cloud,*cloud_out, indices);
	}
	virtual void removeNanNormals(){
		std::vector<int> indices;
		//pcl::removeNaNNormalsFromPointCloud(*cloud,*cloud, indices);
	}
	
	virtual void passThrough(typename pcl::PointCloud<T>::Ptr cloud, typename pcl::PointCloud<T>::Ptr cloud_out, const std::string& axis, int limits1, int limits2){
		pcl::PassThrough<T> pass_x;
		pass_x.setInputCloud (cloud);
		pass_x.setFilterFieldName (axis);
		pass_x.setFilterLimits (limits1, limits2);	
		//pass_x.setFilterLimitsNegative (true);
		pass_x.filter (*cloud_out);
	}
	
	/*Pass through filter**
	virtual void passThrough(const std::string& axis, int limits1, int limits2){
		pcl::PassThrough<T> pass_x;
		pass_x.setInputCloud (_input);
		pass_x.setFilterFieldName (axis);
		pass_x.setFilterLimits (limits1, limits2);	
		//pass_x.setFilterLimitsNegative (true);
		pass_x.filter (*_output);
	}
	
	/*Outlier removal**
	virtual void statisticalOutilerRemoval(typename pcl::PointCloud<T>::Ptr cloud, typename pcl::PointCloud<T>::Ptr cloud_out, int nneighbor, double thresh){
		pcl::StatisticalOutlierRemoval<T> sor;
		sor.setInputCloud (cloud);
		sor.setMeanK (nneighbor);
		sor.setStddevMulThresh (thresh);
		sor.filter (*cloud_out);
	}
	
	/*DownSample*
	virtual void downSample(typename pcl::PointCloud<T>::Ptr cloud, typename pcl::PointCloud<T>::Ptr cloud_out, double radius){
		pcl::PointCloud<int> sampled_indices;

		pcl::UniformSampling<T> uniform_sampling;
		uniform_sampling.setInputCloud (cloud);
		uniform_sampling.setRadiusSearch (radius);
		uniform_sampling.compute (sampled_indices);
		pcl::copyPointCloud (*cloud, sampled_indices.points, *cloud_out);
		
		std::cout << "Shape total points: " << cloud->size () << "; Selected Keypoints: " << cloud_out->size () << std::endl;
	}
	
	
	//************************TESTS*********************
	
	virtual bool gotnanTEST(const typename pcl::PointCloud<T>& cloud){
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
	
	virtual bool gotinfTEST(const typename pcl::PointCloud<T>& cloud){
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
};*/

#endif