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

#include "Pipeline.hpp"

template <typename T, typename DescriptorTypes>
class SegmentAndClustering : public Pipeline<T, DescriptorTypes> {
private:
    //store the pose of the custers
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > _rototranslations;
    std::vector<typename pcl::PointCloud<T> > _clusters;

public:
    //reimplement the virtual function
    virtual void doPipeline();
    //the user interface to get the clusters and their poses
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > getRoto() {
        return _rototranslations;
    }
    std::vector<typename pcl::PointCloud<T>::Ptr> getClusters() {
        return _clusters;
    }
};

template <typename T, typename DescriptorTypes>
void SegmentAndClustering<T, DescriptorTypes>::doPipeline()
{
    typename pcl::PointCloud<T>::Ptr cloud = this->getScene()->getCloud();

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

    //extract each cluster
    pcl::PointCloud<T> cluster;
    for(unsigned int i = 0; i < all_clusters_indices.size(); ++i)
    {
        pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        *indices = all_clusters_indices[i];
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(indices);
        extract.setNegative(false);
        extract.filter(cluster);
        _clusters.push_back(cluster);
    }

    //compute the centroids of each cluster
    Eigen::Matrix< float, 4, 1 > trans;
    _rototranslations.empty();
    for(unsigned int i = 0; i < _clusters.size(); ++i)
    {
	pcl::compute3DCentroid(_clusters[i], trans);
	
	Eigen::Matrix4f pose;
	pose<< 1,0,0,trans[0],
	       0,1,0,trans[1],
	       0,0,1,trans[2],
	       0,0,0,1;
	
	_rototranslations.push_back(pose);
    }
}