#ifndef PIPELINE_RECKON_H
#define PIPELINE_RECKON_H

//Testing Malcolm

#include "Shape3D.hpp"
#include "Shape3DLocal.hpp"
#include "Gui.hpp"
#include <iostream>
#include <stdio.h>
#include <vector>
#include <deque>


template <typename PointType, typename DescriptorType>
class Pipeline{
	
	protected : 
	int _maxObject;
	int _maxScene;
	Shape<PointType, DescriptorType>* _object; //Don't need because it take a Shape* from main.
	Shape<PointType, DescriptorType>* _scene; // Need to be initialise because it takes a Cloud in argument.
	std::vector<Shape<PointType, DescriptorType> *> _objects; //Deck of all objects to compare
	std::vector<Shape<PointType, DescriptorType> *> _scenes; //Deck of all scene to compare
	
	public : 
	Pipeline(Shape<PointType, DescriptorType>* object, Shape<PointType, DescriptorType>* scene) : _maxObject(10), _maxScene(20), _object(object), _scene(scene){}; //FREE PROBLEM
	virtual ~Pipeline(){
		std::cout << "delete Pipeline"<<std::endl;
		delete _object;
		delete _scene;
		std::cout << "delete Pipeline objects top size "<<_objects.size()<<std::endl;
		clearObjects();
		std::cout << "delete Pipeline scene top size "<<_scenes.size()<<std::endl;
		clearScenes();
		
	}
	virtual void doPipeline() = 0;
	virtual void setMaxObject(int i){_maxObject=i;}
	virtual void setMaxScene(int i){_maxScene=i;}
	
	virtual void checkSizeObject();
	virtual void checkSizeScene();
	
	virtual void setObject(Shape<PointType, DescriptorType>* o){std::cout << "set Object shape"<<std::endl; delete _object; _object=o;}
	virtual void setScene(Shape<PointType, DescriptorType>* o){delete _scene; _scene=o;}
	virtual void setObject(typename pcl::PointCloud<PointType>::Ptr& obj);
	virtual void setScene(typename pcl::PointCloud<PointType>::Ptr& cloud);	
	virtual Shape<PointType, DescriptorType>* getObject(){return _object;}
	virtual Shape<PointType, DescriptorType>* getScene(){return _scene;}
	virtual void deleteObject(){delete _object;}
	virtual void deleteScene(){delete _scene;}
	
	//New interface : 
	virtual const std::vector<Shape<PointType, DescriptorType> *>& getAllObjects(){return _objects;}
	virtual const std::vector<Shape<PointType, DescriptorType> *>& getAllScenes(){return _scenes;}
	virtual int getSizeObjects(){ return _objects.size();}
	virtual int getSizeScenes(){ return _scenes.size();}
	virtual Shape<PointType, DescriptorType>* getObject(int i){return _objects[i];}
	virtual Shape<PointType, DescriptorType>* getScene(int i){return _scenes[i];}
	
	virtual void addObject(Shape<PointType, DescriptorType>* o);
	virtual void addScene(Shape<PointType, DescriptorType>* o);
	virtual void addObject(typename pcl::PointCloud<PointType>::Ptr o);
	virtual void addScene(typename pcl::PointCloud<PointType>::Ptr& o);
	
	virtual void removeObject(const std::string& name);
	virtual void removeScene(const std::string& name);
	virtual void removeObject(int i);
	virtual void removeScene(int i);
	virtual void removeObject(Shape<PointType, DescriptorType>* o);
	virtual void removeScene(Shape<PointType, DescriptorType>* o);
	
	virtual void clearObjects();
	virtual void clearScenes();
	virtual void print_info(){
		std::cout<<std::endl<<"********* Objects *********"<<std::endl;
		for(typename std::vector<Shape<PointType, DescriptorType>*>::iterator it = _objects.begin(); it!=_objects.end();){
		//std::cout<<"Cleaning object size : "<<_objects.size()<<std::endl;
			std::cout<<(**it).getName()<<" "<<(**it).getCloud()->size()<<std::endl;
			it++;
		}
		std::cout<<"**************************"<<std::endl;
		std::cout<<std::endl<<"********* Scenes *********"<<std::endl;
		for(typename std::vector<Shape<PointType, DescriptorType>*>::iterator it = _scenes.begin(); it!=_scenes.end();){
		//std::cout<<"Cleaning object size : "<<_objects.size()<<std::endl;
			std::cout<<(**it).getName()<<" "<<(**it).getCloud()->size()<<std::endl;
			it++;
		}
		std::cout<<"**************************"<<std::endl;
		std::cout<<std::endl;
	};
	
	virtual void affiche(){std::cout << "I souldn't exist cause 'm a pipeline with no meaning ;)"<<std::endl;}
	virtual void print(Gui<PointType, DescriptorType>* g){
		g->printPipeline((*this));
	}
};

template <typename T, typename DescriptorType>
inline void Pipeline<T, DescriptorType>::checkSizeObject()
{
	if(_objects.size()>_maxObject){
		while(_objects.size()>_maxObject){
			removeObject(0);
		}
	}
}

template <typename T, typename DescriptorType>
inline void Pipeline<T, DescriptorType>::checkSizeScene()
{
	if(_scenes.size()>_maxScene){
		while(_scenes.size()>_maxScene){
			removeScene(0);
		}
	}
}


template <typename T, typename DescriptorType>
inline void Pipeline<T, DescriptorType>::setScene(typename pcl::PointCloud<T>::Ptr& cloud)
{
	std::cout << "SET THE SCENE"<<std::endl;
	this->_scene->update(cloud);
}

template <typename T, typename DescriptorType>
inline void Pipeline<T, DescriptorType>::setObject(typename pcl::PointCloud<T>::Ptr& obj)
{
	std::cout << "SET THE Object"<<std::endl;
	this->_object->update(obj);
}

template <typename PointType, typename DescriptorType>
inline void Pipeline<PointType, DescriptorType>::addObject(Shape<PointType, DescriptorType>* o){
	_objects.push_back(o);
	checkSizeObject();
	
}
template <typename PointType, typename DescriptorType>
inline void Pipeline<PointType, DescriptorType>::addScene(Shape<PointType, DescriptorType>* o){
	_scenes.push_back(o);
	checkSizeScene();
}

template <typename PointType, typename DescriptorType>
inline void Pipeline<PointType, DescriptorType>::addObject(typename pcl::PointCloud<PointType>::Ptr o){
	std::string name="object";
	name=name+boost::lexical_cast<std::string>( _scenes.size() );
	_objects.push_back(new ShapeLocal<PointType, DescriptorType>(name));
	_objects[_objects.size()-1]->update(o);
	checkSizeObject();
}

template <typename PointType, typename DescriptorType>
inline void Pipeline<PointType, DescriptorType>::addScene(typename pcl::PointCloud<PointType>::Ptr& o){
	std::string name="scene";
	name=name+boost::lexical_cast<std::string>( _scenes.size() );
	_scenes.push_back(new ShapeLocal<PointType, DescriptorType>(name));
	_scenes[_scenes.size()-1]->update(o);
	checkSizeScene();
}

template <typename PointType, typename DescriptorType>
inline void Pipeline<PointType, DescriptorType>::removeObject(const std::string& name){
	for (typename std::vector<Shape<PointType, DescriptorType>*>::iterator it=_objects.begin(); it!=_objects.end();){
		if((**it).getName()==name){
			delete(*it);
			_objects.erase(it);
		}
		else{
			it++;
		}
	}	
}

template <typename PointType, typename DescriptorType>
inline void Pipeline<PointType, DescriptorType>::removeScene(const std::string& name){
	for (typename std::vector<Shape<PointType, DescriptorType>*>::iterator it = _scenes.begin(); it!=_scenes.end();){
		if((**it).getName()==name){
			delete(*it);
			_scenes.erase(it);
		}
		else{
			it++;
		}
	}	
}

template <typename PointType, typename DescriptorType>
inline void Pipeline<PointType, DescriptorType>::removeObject(Shape<PointType, DescriptorType>* o){
	for (typename std::vector<Shape<PointType, DescriptorType>*>::iterator it=_objects.begin(); it!=_objects.end();){
		if((*it)==o){
			delete(*it);
			_objects.erase(it);
		}
		else{
			it++;
		}
	}	
}

template <typename PointType, typename DescriptorType>
inline void Pipeline<PointType, DescriptorType>::removeScene(Shape<PointType, DescriptorType>* o){
	for (typename std::vector<Shape<PointType, DescriptorType>*>::iterator it=_scenes.begin(); it!=_scenes.end();){
		if((*it)==o){
			delete(*it);
			_scenes.erase(it);
		}
		else{
			it++;
		}
	}	
}

template <typename PointType, typename DescriptorType>
inline void Pipeline<PointType, DescriptorType>::removeObject(int i){
	std::cout<<"remove object n "<< i << "with "<< _objects.size()<< std::endl;
	if(size_t(i)<_objects.size()){
		typename std::vector<Shape<PointType, DescriptorType>*>::iterator it=_objects.begin()+i;
		delete(*it);
		_objects.erase(it);
	}
}
	
template <typename PointType, typename DescriptorType>
inline void Pipeline<PointType, DescriptorType>::removeScene(int i){
	std::cout<<"remove scene n "<< i << "with "<< _scenes.size()<< std::endl;
	if(size_t(i)<_scenes.size()){
		typename std::vector<Shape<PointType, DescriptorType>*>::iterator it=_scenes.begin()+i;
		delete(*it);
		_scenes.erase(it);
	}
}

template <typename PointType, typename DescriptorType>
inline void Pipeline<PointType, DescriptorType>::clearObjects(){
	std::cout<<"clear objects"<<std::endl;
	for(typename std::vector<Shape<PointType, DescriptorType>*>::iterator it = _objects.begin(); it!=_objects.end();){
		//std::cout<<"Cleaning object size : "<<_objects.size()<<std::endl;
		delete(*it);
		_objects.erase(it);
	}	
}

template <typename PointType, typename DescriptorType>
inline void Pipeline<PointType, DescriptorType>::clearScenes(){
	std::cout<<"clear scenes"<<std::endl;
	for(typename std::vector<Shape<PointType, DescriptorType>*>::iterator it = _scenes.begin(); it!=_scenes.end();){
		//std::cout<<"Cleaning scene size : "<<_scenes.size()<<std::endl;
		delete(*it);
		_scenes.erase(it);
	}	
}
#endif
	
