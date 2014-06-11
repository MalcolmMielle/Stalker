#ifndef POSTPROCESSING_MALCOLM_H
#define POSTPROCESSING_MALCOLM_H

#include <exception>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <deque>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <time.h>
#include <Shape3D.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <tf/tf.h>
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/LinearMath/Vector3.h"


/*Typical cues are the percentage of supporting points (i.e., model points that are close to scene points), as well as the percentage of outliers (number of visible points belonging to the models that do not have a counterpart within the scene points). Currently, PCL contains an implementation of the hypothesis verification algorithm proposed in [13]. Figure 3 shows an example where the recognition hypotheses are postprocessed using this method. Other verification strategies have been proposed in the literature
 * 
*/
namespace stalker{
	

	void calculatePose(Eigen::Matrix4f& transformation_matrix, geometry_msgs::Pose& pose_in,geometry_msgs::Pose& pose_out);
	
	
	
	template<typename T>
	void voirPCL(typename pcl::PointCloud<T>::Ptr cloud_in, typename pcl::PointCloud<T>::Ptr cloud_out){
		
		pcl::visualization::PCLVisualizer viewer;
		
		pcl::visualization::PointCloudColorHandlerCustom<T>scene_keypoints_color_handler1= pcl::visualization::PointCloudColorHandlerCustom<T> (cloud_in, 0, 0, 255);
		
		pcl::visualization::PointCloudColorHandlerCustom<T>scene_keypoints_color_handler2 = pcl::visualization::PointCloudColorHandlerCustom<T> (cloud_out, 0, 255, 255);
		
		viewer.template addPointCloud<T> (cloud_in,scene_keypoints_color_handler1, "object");
		viewer.template addPointCloud<T> (cloud_out,scene_keypoints_color_handler2, "scene");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "object");	
		while(!viewer.wasStopped()){
			viewer.spinOnce (100);
		}
	}
	
	
	

	
	template<typename T>
	void ICPtransform(typename pcl::PointCloud<T>::Ptr cloud_in, typename pcl::PointCloud<T>::Ptr cloud_out, int &result, double &fitness){
		Eigen::Matrix4f transformation;
		ICPtransform( cloud_in, cloud_out, result, fitness,transformation);
	}
	
	template<typename T>
	void ICPtransform(typename pcl::PointCloud<T>::Ptr cloud_in, typename pcl::PointCloud<T>::Ptr cloud_out, int &result, double &fitness, Eigen::Matrix4f& transformation){
		
		//std::cout<<"Before ICP"<<std::endl;
		//stalker::voirPCL<T>(cloud_in, cloud_out);
		
		
		pcl::IterativeClosestPoint<T, T> icp;
		icp.setInputSource(cloud_in);
		icp.setInputTarget(cloud_out);
		
		pcl::PointCloud<T> Final;
		icp.align(Final);
		result=icp.hasConverged();
		fitness=icp.getFitnessScore();
		transformation=icp.getFinalTransformation();
		//std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		//icp.getFitnessScore() << std::endl;
		//std::cout << icp.getFinalTransformation() << std::endl;	
		
		//std::cout<<"After ICP"<<std::endl;
		//stalker::voirPCL<T>(cloud_in, cloud_out);
	}
	
	template<typename T, typename D>
	void Alignement(Shape<T, D>* object, Shape<T, D>* scene, int result){
		
		// Perform alignment
		pcl::console::print_highlight ("Starting alignment...\n");
		pcl::SampleConsensusPrerejective<T,T,D> align;
		align.setInputSource (object->getCloud());
		align.setSourceFeatures (object->getDescr());
		align.setInputTarget (scene->getCloud());
		align.setTargetFeatures (scene->getDescr());
		align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
		align.setCorrespondenceRandomness (2); // Number of nearest features to use
		align.setSimilarityThreshold (0.6f); // Polygonal edge length similarity threshold
		align.setMaxCorrespondenceDistance (1.5f * object->getSamplingSize()); // Set inlier threshold
		align.setInlierFraction (0.25f); // Set required inlier fraction
	}
	
	void printMatrix4(Eigen::Matrix4f& transformation_matrix){
		printf ("            | %6.3f %6.3f %6.3f %6.3f | \n", transformation_matrix (0,0), transformation_matrix (0,1), transformation_matrix (0,2), transformation_matrix (0,3));
		printf ("        M = | %6.3f %6.3f %6.3f %6.3f | \n", transformation_matrix (1,0), transformation_matrix (1,1), transformation_matrix (1,2), transformation_matrix (1,3));
		printf ("            | %6.3f %6.3f %6.3f %6.3f | \n", transformation_matrix (2,0), transformation_matrix (2,1), transformation_matrix (2,2), transformation_matrix (2,3));
		printf ("            | %6.3f %6.3f %6.3f %6.3f | \n", transformation_matrix (3,0), transformation_matrix (3,1), transformation_matrix (3,2), transformation_matrix (3,3));
	}
	
	void printMatrix3(Eigen::Matrix3f& transformation_matrix){
		printf ("            | %6.3f %6.3f %6.3f | \n", transformation_matrix (0,0), transformation_matrix (0,1), transformation_matrix (0,2));
		printf ("        M = | %6.3f %6.3f %6.3f | \n", transformation_matrix (1,0), transformation_matrix (1,1), transformation_matrix (1,2));
		printf ("            | %6.3f %6.3f %6.3f | \n", transformation_matrix (2,0), transformation_matrix (2,1), transformation_matrix (2,2));

		
	}
	
	//TODO To test
	void calculatePose(Eigen::Matrix4f& transformation_matrix, geometry_msgs::Pose& pose_in,geometry_msgs::Pose& pose_out ){
		
		tf::Pose pose_tf_in;
		tf::poseMsgToTF(pose_in, pose_tf_in);
		
		//Conversion of eigen
		Eigen::Matrix3d mat;
		mat(0,0)= transformation_matrix(0,0);
		mat(0,1)= transformation_matrix(0,1);
		mat(0,2)= transformation_matrix(0,2);
		mat(1,0)= transformation_matrix(1,0);
		mat(1,1)= transformation_matrix(1,1);
		mat(1,2)= transformation_matrix(1,2);
		mat(2,0)= transformation_matrix(2,0);
		mat(2,1)= transformation_matrix(2,1);
		mat(2,2)= transformation_matrix(2,2);
		
		//GET QUATERNIONS
		tf::Matrix3x3 rot;
		tf::matrixEigenToTF(mat, rot);
		
		tf::Quaternion tfqt_in;
		tf::Quaternion tfqt_cal;
		
		rot.getRotation(tfqt_cal);
		tfqt_in=pose_tf_in.getRotation();
		
		tfqt_in*=tfqt_cal; //final
		
		//GET TRANSLATIONS
		tf::Vector3 vec;
		tf::Vector3 vec_in;
		vec.setValue(transformation_matrix(0,3), transformation_matrix(1,3), transformation_matrix(2,3));
		vec_in=(pose_tf_in.getOrigin());
		
		vec=vec_in+vec;

		
		//TODO SOLVE THIS
		pose_out.position.x=vec.getX();
		pose_out.position.y=vec.getY();
		pose_out.position.z=vec.getZ();
			
		
		pose_out.orientation.x=tfqt_in.getX();
		pose_out.orientation.y=tfqt_in.getY();
		pose_out.orientation.z=tfqt_in.getZ();
		pose_out.orientation.x=tfqt_in.getW();

	}
	


}

#endif