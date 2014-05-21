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
#include <Preprocessing.hpp>



template <typename T,typename DescriptorType>
class Main{
	protected : 
		
	int _whichInterface;
	int _maxObject;
	int _maxScene;
	bool resol_state;
	double _resolution;
	
	typename pcl::PointCloud<T>::Ptr _scene; //Last Scene receive
	typename pcl::PointCloud<T>::Ptr _object; //Last Object receive
	
	std::vector<typename pcl::PointCloud<T>::Ptr> _objects; //Beta List of all objects receive
	std::vector<typename pcl::PointCloud<T>::Ptr> _scenes; //Beta List of all scene receive
	
	/*I chose the point cloud because it's easier to have something general of use if we receive a Point Cloud through Ros nodes*/
	Pipeline<T, DescriptorType>* _pipeline;

	
	public : 
	
	/********DEFAULT CONSTRCTOR***********/
	Main() : _whichInterface(1), _maxObject(10), _maxScene(20),resol_state(false),_resolution(0),
	_scene(new pcl::PointCloud<T>()), 
	_object(new pcl::PointCloud<T>()), 
	_pipeline(new CorrespGrouping<T, DescriptorType>(new ShapeLocal<T, DescriptorType>("object"), new ShapeLocal<T, DescriptorType>("scene"))) 
	{
		std::cout<<"buiding the main awith nothing"<<std::endl;

	}

	/********you have the Clouds CONSTRCTOR***********/
	Main(typename pcl::PointCloud<T>::Ptr& object, typename pcl::PointCloud<T>::Ptr& scene) : _whichInterface(1), _maxObject(10), _maxScene(20),resol_state(false),_resolution(0),
	_scene(scene), 
	_object(object), 
	_pipeline(new CorrespGrouping<T, DescriptorType>(new ShapeLocal<T, DescriptorType>("object"), new ShapeLocal<T, DescriptorType>("scene")))
	{
		std::cout<<"buiding the main"<<std::endl;
		initPipeline(); //Mem leak
	}
	
	/********you have the Pipeline CONSTRCTOR***********/
	Main(Pipeline<T, DescriptorType>* p) : _whichInterface(1), _maxObject(10), _maxScene(20),resol_state(false),_resolution(0),
	_scene(new pcl::PointCloud<T>()), 
	_object(new pcl::PointCloud<T>()), 
	_pipeline(p)
	{
		std::cout<<"buiding the main with just a pipeline"<<std::endl;
	}

	/********You have everything CONSTRCTOR***********/
	Main(typename pcl::PointCloud<T>::Ptr& object, typename pcl::PointCloud<T>::Ptr& scene, Pipeline<T, DescriptorType>* p) : _whichInterface(1), _maxObject(10), _maxScene(20),resol_state(false),_resolution(0),
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
	
	virtual typename pcl::PointCloud<T>::Ptr getCloud(){return _scene;}
	virtual typename pcl::PointCloud<T>::Ptr getShape(){return _object;}
	virtual Pipeline<T, DescriptorType>* getPipeline(){return _pipeline;}
	virtual int getInterface(){return _whichInterface;}
	
	virtual void setResolution(bool b){resol_state=b;_pipeline->getObject()->setResolutionState(b);_pipeline->getScene()->setResolutionState(b);}
	virtual void setScene(typename pcl::PointCloud<T>::Ptr& c);
	virtual void setObject(typename pcl::PointCloud<T>::Ptr& s);
	virtual void setPipeline(Pipeline<T, DescriptorType>* p){delete _pipeline; _pipeline=p;}
	virtual void setInterface(int i){
		try{
			if(i==1 || i==2){
				_whichInterface=i;
			}
			else{
				throw std::invalid_argument("no Interface selected in Main. _whichInterface must be either 1 for using 1 point cloud and 1 object old interface or 2 for using the vector of scene and model interface. Interface number 1 should be erased at some point but it still supported for now");//Need to figure out how to change and know the shape's names !! Maybe a yaml file...
			}
		}
		catch(std::exception const& e){
			std::cerr << "ERREUR in Main in stalker : " << e.what() << std::endl;
			exit(0);
		}
	}
		
		
		
	//New interface
	virtual void addObject(typename pcl::PointCloud<T>::Ptr& c){_objects.push_back(c);checkSizeObject();_pipeline->addObject(c);}
	virtual void addScene(typename pcl::PointCloud<T>::Ptr& c){_scenes.push_back(c);checkSizeScene();_pipeline->addScene(c);}
	
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
	
		virtual void setResolutionState(bool b){ resol_state=b;}
	virtual void setResolution(double r){_resolution=r;}
		virtual double getResolutionState(){return resol_state;}
	virtual double getResolution(){return _resolution;}
	
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
	/**************************************************/
	//FIX THE RESOLUTION FROM THE MODEL HERE !//
	double reso;
	if(resol_state==true){
		reso=stalker::computeCloudResolution<T>(_object);
	}
	
	
	/**************************************************/
	if(_whichInterface==1){
		_pipeline->setObject(_object);
		if(resol_state==true){
			_pipeline->getObject()->setResolution(reso);
		}
	}
	else{
		addObject(_object);
	}
}


template <typename T, typename DescriptorType>
inline void Main<T, DescriptorType>::loadModel(const typename pcl::PointCloud<T>::Ptr cloudy){
	
	_object=cloudy;
	/**************************************************/
	//FIX THE RESOLUTION FROM THE MODEL HERE !//
	double reso;
	if(resol_state==true){
		reso=stalker::computeCloudResolution<T>(_object);
	}
	
	
	/**************************************************/
	if(_whichInterface==1){
		_pipeline->setObject(_object);
		if(resol_state==true){
			_pipeline->getObject()->setResolution(reso);
		}
	}
	else{
		addObject(_object);
	}

}

/**********************DO WORK FUNCTIONS***************/

/*Function to modify in order to use one "interface" or the other*/


template <typename T, typename DescriptorType>
inline void Main<T, DescriptorType>::doWork(const sensor_msgs::PointCloud2ConstPtr& cloudy){
	pcl::fromROSMsg(*cloudy, *_scene);
	
	/**************************************************/
	//TAKE THE RESOLUTION FROM THE MODEL HERE !//
	double reso;
	if(resol_state==true){
		reso=_pipeline->getObject()->getResolution();
	}
	
	/**************************************************/
	
	if(_whichInterface==1){
		_pipeline->setScene(_scene);
		if(resol_state==true){
			_pipeline->getScene()->setResolution(reso);
		}
	}
	else{
		addScene(_scene);
	}
	
	doWork();
}

template <typename T, typename DescriptorType>
inline void Main<T, DescriptorType>::doWork(){
	std::cout<<"doWork"<<std::endl;
	//TODO processing here
	
	/**************************MAIN PIPELINE OF RECOGNITION**********************/
	if(_whichInterface==1){
		if(_object->width>0 && _object->height>0 &&_scene->width>0 && _scene->height>0){
			_pipeline->doPipeline();
			_pipeline->affiche();
		}
	}
	else{
		if(_pipeline->getAllObjects().size()>0 && _pipeline->getAllScenes().size()>0){
			_pipeline->doPipeline();
			_pipeline->affiche();
		}
	}

	//TODO Post Processing here

}





#endif
