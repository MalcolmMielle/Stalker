#include <iostream>

#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

template<typename T>
class Preprocessing 
{ 
protected : 
        
public : 
        Preprocessing(){}; 
                    
        void removeNan(typename pcl::PointCloud<T>::Ptr& cloud) 
        { 
                std::vector<int> indices; 
                pcl::removeNaNFromPointCloud(*cloud,*cloud, indices); 
        } 
                    
        void removeNanNormals(typename pcl::PointCloud<T>::Ptr& cloud) 
        { 
                std::vector<int> indices; 
                pcl::removeNaNNormalsFromPointCloud(*cloud,*cloud, indices); 
        } 
}; 

int 
main (int argc, char** argv) 
{ 
        Preprocessing<pcl::PointXYZRGBA> test; 
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>() );
	test.removeNanNormals(cloud);
	test.removeNan(cloud);
        return (0); 
}