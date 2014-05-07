#ifndef GUI1_MALCOLM_H
#define GUI1_MALCOLM_H

#include <pcl/common/transforms.h>

#include "Gui.hpp"
#include "Shape3DLocal.hpp"
#include "CorrespGrouping.hpp"
template <typename T, typename DescriptorType>
class Gui1 : public Gui<T, DescriptorType>{
	
	public : 
	pcl::visualization::PointCloudColorHandlerCustom<T> scene_keypoints_color_handler;
	
	Gui1() : 
	Gui<T, DescriptorType>(), 
	scene_keypoints_color_handler(typename pcl::PointCloud<T>::Ptr(new pcl::PointCloud<T>()), 0, 0, 255){}
		
		
	virtual void add(typename pcl::PointCloud<T>::Ptr cloud, std::string name){
		std::cout<<"Adding Point Cloud"<<std::endl;
		this->viewer->template addPointCloud<T> (cloud, name);
		this->viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name);
	}		
	
	virtual void update(typename pcl::PointCloud<T>::Ptr cloud, std::string name){
		this->viewer->updatePointCloud(cloud, name);
	}
	
	virtual void show(){
		this->viewer->spinOnce (100);
	}	
	
	virtual void add(Shape<T, DescriptorType>& sh){
		std::cout<<"Adding shape"<<std::endl;
		sh.addPrint(*this);
	}
	
	virtual void update(Shape<T, DescriptorType>& sh){
		std::cout<<"Adding shape"<<std::endl;
		sh.printupdate(*this);
	}
	
	virtual void update(ShapeLocal<T, DescriptorType>& sh){
		update(sh.getCloud(), sh.getName());
		std::string name=sh.getName()+"keypoints";
		update(sh.getKeypoints(), name);
	}
	
	virtual void add(ShapeLocal<T, DescriptorType>& sh){
		//sh->addPrint(this);
		std::cout<<"Adding shape Local "<< sh.getName()<<std::endl;
		this->add(sh.getCloud(), sh.getName());
		printKeyPoints(sh);
	}
	
	virtual void printPipeline(Pipeline<T, DescriptorType>& p){
		p.print(this);
	}
	
	virtual void printKeyPoints(ShapeLocal<T, DescriptorType>& sh){
		std::cout<<"Adding kaypoints"<<std::endl;
		std::string name=sh.getName()+"keypoints";
		scene_keypoints_color_handler = pcl::visualization::PointCloudColorHandlerCustom<T> (sh.getKeypoints(), 0, 0, 255);
		this->viewer->template addPointCloud<T>(sh.getKeypoints(), scene_keypoints_color_handler, name);
		this->viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
	}
	
	virtual void printPipeline(CorrespGrouping<T>& sh){
		
		typename pcl::PointCloud<T>::Ptr off_scene_model_keypoints (new pcl::PointCloud<T> ());

		//pcl::transformPointCloud (*(sh.getObject()->getKeypoints()), *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
		off_scene_model_keypoints=sh.getObject()->getKeypoints();
		
		std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations = sh.getRoto();
		std::vector<pcl::Correspondences> clustered_corrs=sh.getClust();
		typename pcl::PointCloud<T>::Ptr model = sh.getObject()->getCloud();
		typename pcl::PointCloud<T>::Ptr model_keypoints=sh.getObject()->getKeypoints();
		
		typename pcl::PointCloud<T>::Ptr scene = sh.getScene()->getCloud();
		typename pcl::PointCloud<T>::Ptr scene_keypoints=sh.getScene()->getKeypoints();
		
		for (size_t i = 0; i < rototranslations.size (); ++i)
		{
			typename pcl::PointCloud<T>::Ptr rotated_model (new typename pcl::PointCloud<T> () );
			pcl::transformPointCloud (*(sh.getObject()->getCloud()), *rotated_model, rototranslations[i]);

			std::stringstream ss_cloud;
			ss_cloud << "instance" << i;

			pcl::visualization::PointCloudColorHandlerCustom<T> rotated_model_color_handler (rotated_model, 255, 0, 0);
			this->viewer->addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

			for (size_t j = 0; j < clustered_corrs[i].size (); ++j)
			{
				std::stringstream ss_line;
				ss_line << "correspondence_line" << i << "_" << j;
				T& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
				T& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

				//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
				this->viewer->addLine (model_point, scene_point, 0, 255, 0, ss_line.str ());
			}
		}
	
	}
	
};

#endif