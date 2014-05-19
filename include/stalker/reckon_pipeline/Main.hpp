#ifndef RECKON_MAIN_OPEN_H
#define RECKON_MAIN_OPEN_H

#include <exception>
#include <iostream>
#include <stdio.h>
#include "sensor_msgs/PointCloud2.h"
#include <vector>
#include <Shape3DLocal.hpp>
#include <CorrespGrouping.hpp>
#include <Shape3D.hpp>
#include "Pipeline.hpp"
#include <pcl/filters/filter.h>
#include "Preprocessing.hpp"


template <typename T,typename DescriptorType>
class Main{
	protected : 
	int _maxObject;
	int _maxScene;
	typename pcl::PointCloud<T>::Ptr _scene; //Last Scene receive
	typename pcl::PointCloud<T>::Ptr _object; //Last Object receive
	
	std::vector<typename pcl::PointCloud<T>::Ptr> _objects; //Beta List of all objects receive
	std::vector<typename pcl::PointCloud<T>::Ptr> _scenes; //Beta List of all scene receive
	
	/*I chose the point cloud because it's easier to have something general of use if we receive a Point Cloud through Ros nodes*/
	Pipeline<T, DescriptorType>* _pipeline;
	Preprocessing<T> _prep;
	
	public : 
	
	/********DEFAULT CONSTRCTOR***********/
	Main() : _maxObject(10), _maxScene(20),
	_scene(new pcl::PointCloud<T>()), 
	_object(new pcl::PointCloud<T>()), 
	_pipeline(new CorrespGrouping<T, DescriptorType>(new ShapeLocal<T, DescriptorType>("object"), new ShapeLocal<T, DescriptorType>("scene"))) 
	{
		std::cout<<"buiding the main awith nothing"<<std::endl;

	}

	/********you have the Clouds CONSTRCTOR***********/
	Main(typename pcl::PointCloud<T>::Ptr& object, typename pcl::PointCloud<T>::Ptr& scene) : _maxObject(10), _maxScene(20),
	_scene(scene), 
	_object(object), 
	_pipeline(new CorrespGrouping<T, DescriptorType>(new ShapeLocal<T, DescriptorType>("object"), new ShapeLocal<T, DescriptorType>("scene")))
	{
		std::cout<<"buiding the main"<<std::endl;
		initPipeline(); //Mem leak
	}
	
	/********you have the Pipeline CONSTRCTOR***********/
	Main(Pipeline<T, DescriptorType>* p) : _maxObject(10), _maxScene(20),
	_scene(new pcl::PointCloud<T>()), 
	_object(new pcl::PointCloud<T>()), 
	_pipeline(p)
	{
		std::cout<<"buiding the main with just a pipeline"<<std::endl;
	}

	/********You have everything CONSTRCTOR***********/
	Main(typename pcl::PointCloud<T>::Ptr& object, typename pcl::PointCloud<T>::Ptr& scene, Pipeline<T, DescriptorType>* p) : _maxObject(10), _maxScene(20),
	_scene(scene), 
	_object(object), 
	_pipeline(p)
	{
		std::cout<<"buiding the main"<<std::endl;
		initPipeline();
	}
	
	/*********************DESTRUCTOR*********************/
	virtual ~Main(){
		std::cout<<"deleting the main"<<std::endl;
		delete _pipeline;
		clearObjects();
		clearScenes();
	}

	
	/***********Init***********************/
	void init(typename pcl::PointCloud<T>::Ptr& object, typename pcl::PointCloud<T>::Ptr& scene){
		setScene(scene);
		setObject(object);
	}
	
	void initPipeline(typename pcl::PointCloud<T>::Ptr& shape, typename pcl::PointCloud<T>::Ptr& scene){
		//_pipeline->init(cloud, cloud2);
		_pipeline->setObject(shape);
		_pipeline->setScene(scene);
	}
	
	void initPipeline(){
		//_pipeline->init(cloud, cloud2);
		_pipeline->setObject(_object); //Mem leak
		_pipeline->setScene(_scene);
	}
	
	//Accesseur
	virtual void setMaxObject(int i){_maxObject=i;}
	virtual void setMaxScene(int i){_maxScene=i;}
	
	virtual void checkSizeObject();
	virtual void checkSizeScene();
	
	typename pcl::PointCloud<T>::Ptr getCloud(){return _scene;}
	typename pcl::PointCloud<T>::Ptr getShape(){return _object;}
	Pipeline<T, DescriptorType>* getPipeline(){return _pipeline;}
	
	virtual void setScene(typename pcl::PointCloud<T>::Ptr& c);
	virtual void setObject(typename pcl::PointCloud<T>::Ptr& s);
	virtual void setPipeline(Pipeline<T, DescriptorType>* p){delete _pipeline; _pipeline=p;}
	
	//New interface
	virtual void addObject(typename pcl::PointCloud<T>::Ptr& c){_objects.push_back(c);checkSizeObject();_pipeline->addObject(c);}
	virtual void addScene(typename pcl::PointCloud<T>::Ptr& c){_scenes.push_back(c);checkSizeScene();_pipeline->addScene(c);
		
	}
	
	virtual void removeObject(typename pcl::PointCloud<T>::Ptr& c);
	virtual void removeScene(typename pcl::PointCloud<T>::Ptr& c);
	virtual void removeObject(int i);
	virtual void removeScene(int i);
	
	virtual void clearObjects();
	virtual void clearScenes();
	const std::vector<typename pcl::PointCloud<T>::Ptr>& getAllObjects(){return _objects;}
	const std::vector<typename pcl::PointCloud<T>::Ptr>& getAllScenes(){return _scenes;}
	
	virtual void loadModel(const sensor_msgs::PointCloud2ConstPtr& cloudy);
	virtual void loadModel(const typename pcl::PointCloud<T>::Ptr cloudy);
	virtual void doWork(const sensor_msgs::PointCloud2ConstPtr& cloudy); 
	virtual void doWork();
	
};


template <typename T, typename DescriptorType>
inline void Main<T, DescriptorType>::checkSizeObject()
{
	if(_objects.size()>_maxObject){
		while(_objects.size()>_maxObject){
			removeObject(0);
		}
	}
}

template <typename T, typename DescriptorType>
inline void Main<T, DescriptorType>::checkSizeScene()
{
	if(_scenes.size()>_maxScene){
		while(_scenes.size()>_maxScene){
			removeScene(0);
		}
	}
}



template <typename T, typename DescriptorType>
inline void Main<T, DescriptorType>::setScene(typename pcl::PointCloud<T>::Ptr& c){
	this->_scene=c;  
	this->_pipeline->setScene(_scene);
}
template <typename T, typename DescriptorType>
inline void Main<T, DescriptorType>::setObject(typename pcl::PointCloud<T>::Ptr& s){
	this->_object=s; 
	this->_pipeline->setObject(_object);
}

template <typename T, typename DescriptorType>
inline void Main<T, DescriptorType>::removeObject(typename pcl::PointCloud<T>::Ptr& c){
	for (typename std::vector<typename pcl::PointCloud<T>::Ptr>::iterator it = _objects.begin(); it!=_objects.end();){
		if((*it)==c){
			//delete(*it);//I think this is wrong because it's not a pointer...
			_objects.erase(it);
		}
		else{
			it++;
		}
	}
}

template <typename T, typename DescriptorType>
inline void Main<T, DescriptorType>::removeScene(typename pcl::PointCloud<T>::Ptr& c){
	for (typename std::vector<typename pcl::PointCloud<T>::Ptr>::iterator it = _scenes.begin(); it!=_scenes.end();){
		if((*it)==c){
			//delete(*it);//Same as above
			_scenes.erase(it);
		}
		else{
			it++;
		}
	}
}

template <typename T, typename DescriptorType>
inline void Main<T, DescriptorType>::removeObject(int i){
	std::cout<<"remove object n "<< i << "with "<< _objects.size()<< std::endl;
	if(size_t(i)<_objects.size()){
		typename std::vector<typename pcl::PointCloud<T>::Ptr>::iterator it=_objects.begin()+i;
		_objects.erase(it);
	}
}

template <typename T, typename DescriptorType>
inline void Main<T, DescriptorType>::removeScene(int i){
	std::cout<<"remove scene n "<< i << "with "<< _scenes.size()<< std::endl;
	if(size_t(i)<_scenes.size()){
		typename std::vector<typename pcl::PointCloud<T>::Ptr>::iterator it=_scenes.begin()+i;
		_scenes.erase(it);
	}
}

template <typename T, typename DescriptorType>
inline void Main<T, DescriptorType>::clearObjects(){
	for(typename std::vector<typename pcl::PointCloud<T>::Ptr>::iterator it = _objects.begin(); it!=_objects.end();){
		_objects.erase(it);
	}	
}

template <typename T, typename DescriptorType>
inline void Main<T, DescriptorType>::clearScenes(){
	for(typename std::vector<typename pcl::PointCloud<T>::Ptr>::iterator it = _scenes.begin(); it!=_scenes.end();){
		_scenes.erase(it);
	}
}

/**********************LOAD MODEL FUNCTION**************/
template <typename T, typename DescriptorType>
inline void Main<T, DescriptorType>::loadModel(const sensor_msgs::PointCloud2ConstPtr& cloudy){
	pcl::fromROSMsg(*cloudy, *_object);
	//Preprocessing
	try{
		bool nany= _prep.gotnanTEST(_object);
		if(nany==true || _object->is_dense==false){
			_object->is_dense=false;
			_prep.removeNan(_object);
			_prep.removeNanNormals(_object);
			
			std::cout<<"remove dense from model"<<std::endl;
			
			if(_prep.gotnanTEST(_object)) throw std::invalid_argument("not dense : nan");
			if(_prep.gotinfTEST(_object)) throw std::invalid_argument("not dense : inf");
			
			_object->is_dense=true;
		}
		else{std::cout<<"DENSE :D"<<std::endl;}
	}
	catch(std::exception const& e){
		std::cerr << "ERREUR model is not dense : " << e.what() << std::endl;	
	}
	
	std::cout<<"setObject"<<std::endl;
	_pipeline->setObject(_object);
	std::cout<<"addObject"<<std::endl;
	addObject(_object);
}


template <typename T, typename DescriptorType>
inline void Main<T, DescriptorType>::loadModel(const typename pcl::PointCloud<T>::Ptr cloudy){
	
	_object=cloudy;
	
	//PREPROCESSING
	try{
		//TOO CHANGE !!
		bool nany= _prep.gotnanTEST(_object);
		
		if(nany==true || _object->is_dense==false){
			_object->is_dense=false;
			_prep.removeNan(_object);
			_prep.removeNanNormals(_object);
			
			std::cout<<"remove dense from model"<<std::endl;
			
			if(_prep.gotnanTEST(_object)) throw std::invalid_argument("not dense : nan");
			if(_prep.gotinfTEST(_object)) throw std::invalid_argument("not dense : inf");
			
			_object->is_dense=true;
		}
		else{std::cout<<"DENSE :D"<<std::endl;}
	}
	catch(std::exception const& e){
		std::cerr << "ERREUR model is not dense : " << e.what() << std::endl;	
	}
	
	_pipeline->setObject(_object);
	addObject(_object);
}

/**********************DO WORK FUNCTIONS***************/

/*Function to modify in order to use one "interface" or the other*/


template <typename T, typename DescriptorType>
inline void Main<T, DescriptorType>::doWork(const sensor_msgs::PointCloud2ConstPtr& cloudy){
	pcl::fromROSMsg(*cloudy, *_scene);
	if(_scene->is_dense==false){
		std::cout<<"NOT DENNNNNNNSE"<<std::endl;
	}
	/****PREPROCESSING****/
	
	//Remove NANS
	try{
		bool nany= _prep.gotnanTEST(_scene);
		if(nany==true || _scene->is_dense==false){
			_scene->is_dense=false;
			_prep.removeNan(_scene);
			_prep.removeNanNormals(_scene);
			
			std::cout<<"remove dense from scene"<<std::endl;
			
			if(_prep.gotnanTEST(_scene)) throw std::invalid_argument("not dense : nan");
			if(_prep.gotinfTEST(_scene)) throw std::invalid_argument("not dense : inf");
			
			_scene->is_dense=true;
		}
		else{std::cout<<"DENSE :D"<<std::endl;}
	}
	catch(std::exception const& e){
		std::cerr << "ERREUR scene is not dense : " << e.what() << std::endl;	
	}
	
	
	std::cout<<_scene->size() <<std::endl;
	_pipeline->setScene(_scene);
	addScene(_scene); //Need to figure out how to change and know the shape's names !! Maybe a yaml file...
	doWork();
}

template <typename T, typename DescriptorType>
inline void Main<T, DescriptorType>::doWork(){
	std::cout<<"doWork"<<std::endl;
	//TODO processing here
	
	/**************************MAIN PIPELINE OF RECOGNITION**********************/
	
	_pipeline->doPipeline();
	_pipeline->affiche();
	
	exit(0);
	
	//TODO Post Processing here

}


#endif
