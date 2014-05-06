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
	
	~MainGraphic(){std::cout<<"deleting the mainGraphic"<<std::endl;
	delete gui;}
	//Accesseur
	virtual Gui<T, DescriptorType>* getGui(){return gui;}
	bool stopGui(){return gui->viewer->wasStopped ();}
	//virtual void doWork(const sensor_msgs::PointCloud2ConstPtr& cloudy); 
	virtual void doWork();
	
};

/*void MainGraphic::doWork(const sensor_msgs::PointCloud2ConstPtr& cloudy)
{
	Main::doWork(cloudy);
}*/
template < typename T, typename DescriptorType>
void MainGraphic<T, DescriptorType>::doWork()
{
	Main<T, DescriptorType>::doWork();
	this->gui->update(*(this->_pipeline->getObject() ));
	this->gui->update(*(this->_pipeline->getScene() ));
	this->gui->printPipeline(*(this->_pipeline));
	this->gui->show();
}


#endif