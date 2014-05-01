#ifndef RECKON_MAINGRAPHIC_H
#define RECKON_MAINGRPAHIC_H

#include "Main.hpp"
#include "Gui.hpp"

template < typename T,typename DescriptorType>
class MainGraphic : public Main<T, DescriptorType>{
	protected :
	Gui<T>* gui;
	
	public :
	MainGraphic() : Main<T, DescriptorType>(), gui(new Gui<T>){
		std::cout<<"buiding the main Graphic"<<std::endl;
		gui->add(this->_cloud,"scene");
	}
	
	~MainGraphic(){std::cout<<"deleting the mainGraphic"<<std::endl;
	delete gui;}
	//Accesseur
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
	gui->update(this->_cloud, "scene");
	gui->show();
}


#endif