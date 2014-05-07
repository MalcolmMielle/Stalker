#ifndef RECKON_MAINGRAPHIC_H
#define RECKON_MAINGRPAHIC_H

#include "Main.hpp"
#include "Gui1.hpp"

template < typename T,typename DescriptorType>
class MainGraphic : public Main<T, DescriptorType>{
	protected :
	Gui<T, DescriptorType>* gui;
	
	public :
	MainGraphic() : Main<T, DescriptorType>(), gui(new Gui1<T, DescriptorType>){
		std::cout<<"buiding the main Graphic"<<std::endl;
		//gui->add(this->_cloud,"scene");
		gui->add(*(this->_pipeline->getObject() ));
		gui->add(*(this->_pipeline->getScene() ));
	}
	
	MainGraphic(typename pcl::PointCloud<T>::Ptr& object, typename pcl::PointCloud<T>::Ptr& scene) : Main<T, DescriptorType>(object, scene), gui(new Gui1<T, DescriptorType>){
		std::cout<<"buiding the main Graphic"<<std::endl;
		//gui->add(this->_cloud,"scene");
		gui->add(*(this->_pipeline->getObject() ));
		gui->add(*(this->_pipeline->getScene() ));
	}
	
	~MainGraphic(){
		std::cout<<"deleting the mainGraphic"<<std::endl;
		//Main<T, DescriptorType>::~Main();
		delete gui;}
	//Accesseur
	virtual Gui<T, DescriptorType>* getGui(){return gui;}
	bool stopGui(){return gui->viewer->wasStopped ();}
	//virtual void doWork(const sensor_msgs::PointCloud2ConstPtr& cloudy); 
	virtual void doWork();
	//OVERWRITE FUNCTIONS
	
	void addObject(typename pcl::PointCloud<T>::Ptr& c){
		this->_objects.push_back(c);
		this->checkSizeObject();
		this->_pipeline->addObject(c);
		std::cout<<"ADD INTO THE GRAPHICS"<<std::endl<<std::endl;
		gui->add(*(this->_pipeline->getObject( (this->_pipeline->getSizeObjects())-1) ) );
	}	
	void addScene(typename pcl::PointCloud<T>::Ptr& c){
		this->_scenes.push_back(c);
		this->checkSizeScene();
		this->_pipeline->addScene(c);
		gui->add(*(this->_pipeline->getScene( (this->_pipeline->getSizeScenes())-1) ) );
	}
	
};

/*void MainGraphic::doWork(const sensor_msgs::PointCloud2ConstPtr& cloudy)
{
	Main::doWork(cloudy);
}*/
template < typename T, typename DescriptorType>
void MainGraphic<T, DescriptorType>::doWork()
{
	//Main<T, DescriptorType>::doWork();
	this->gui->update(*(this->_pipeline->getObject() ));
	this->gui->update(*(this->_pipeline->getScene() ));
	
	for (int i=0; i< this->_pipeline->getSizeObjects();i++){
		std::cout<<"Getting do Work into the objectS because we have " << this->_pipeline->getSizeObjects()<<" objects"<<std::endl;
		this->gui->update(*(this->_pipeline->getObject(i)));
	}
	for (int i=0; i< this->_pipeline->getSizeScenes();i++){
		std::cout<<"Getting do Work into the sceneS because we have "<<this->_pipeline->getSizeScenes()<<" scenes" <<std::endl;
		this->gui->update(*(this->_pipeline->getScene(i)));
	}
	
	this->gui->printPipeline(*(this->_pipeline));
	this->gui->show();
}


#endif