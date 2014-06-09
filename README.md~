<pre>      _/_/_/  _/_/_/_/_/    _/_/    _/        _/    _/  _/_/_/_/  _/_/_/    
   _/            _/      _/    _/  _/        _/  _/    _/        _/    _/   
    _/_/        _/      _/_/_/_/  _/        _/_/      _/_/_/    _/_/_/      
       _/      _/      _/    _/  _/        _/  _/    _/        _/    _/     
_/_/_/        _/      _/    _/  _/_/_/_/  _/    _/  _/_/_/_/  _/    _/      
     
</pre>                                                                      
Welcome
=======

                                                                             
Welcome at you, using the stalker =).

The stalker is an interface made to use 3D Real Time recognition algorithms under [ROS](http://www.ros.org/).

You can find in this directory an UML of the Stalker so it's easy for you to use he interface to adapt it in your own code.

Globally, the Stalker is ensemble of three *main* class : 

* Shape
* Main
* Pipeline

Except for the Main class, all other class are abstract class meaning you have to actually implement them.

Class Description
==================

Shape
-----

The __Shape__ class is the most basic class of the Stalker. It is used to discribe a Point Cloud by computing a bunch a descriptor. For now, the descriptor used is the SHOT one but That class should soon be templated so you can easily change it depending on your needs. For now, if you want to change it their is a typedef on top a the Shape3D.hpp file were you just have to redefine DescriptorType.
The abstract function _update(PointCloud::Ptr)_ is the one you need to implement. It's the one that compute the descriptor of your point Cloud.


Main
----


The __Main__ Class is the one that receive information as a Point Cloud Pointer from the _Stalker Node_. It pass the information to the __Pipeline__ class while converting it to Shape.

Pipeline
---------

__Pipeline__ is supposed to be your recognition pipeline.
You need to implement the function _doPipeline()_ which is going to be called by __Main__ each time a message is receive through ROS.

Main and Pipeline interface
============================

You can use two different interfaces. One use only one scene and one model to compute while the other one use a vector of both.
This allow you to choose if you need to find multiple models or not for example. Or if you have multiple scene (we never know you could have multiples camera). I let the "old" implementation with one scene and one model because it might be faster depending on the usage. To choose which interface to use just go in the _doWork()_ function of __Main__ and either use _addScene()_ and _addObject()_ to use the vector interface or _setScene()_ and _setObject()_ if you want to use the "only one model and scene" interface.

WARNING : the vector interface is totally experimental and buggy for now since I hadn't had to use it. Especially when used with a graphical interface, it need to be tested and revised !!

Like this, _Pipeline_ receive only __Shapes__


Examples
========

With this repository, you'll find different class, descriptors and Pipeline.

Here is a list of them :

* Class : 
  + ShapeLocal : Shape implementing local descriptors
  + ShapeModelSlowCalculation : shape implementing local descriptors. Same as ShapeLocal but the downsample is better since we are using the Uniform Grid as describe in **A New Method for Cloud Data Reduction Using Uniform Grids** by __Zhao Sanyuan, Li Fengxia, Liu Yongmei, Rao Yonghui__. Thus the calculation are slower. It's useful if the model is never going to change in time.
  + MainGraphic : Graphic version of the Main class
  + Gui1 : First extremely simple version of a Gui.

* Descriptors : 
  + SHOT352
  + SHOT1337 : Color information.
  + SpinImage
  + FPFH (not tested !) 

* Pipeline : 
  + CorrespondenceGrouping
  

Nodes
========

Here is a description of all the nodes available : 

* Create model => Node that take a point cloud in input and save it as a pcd file(__view_model.pcd__) which is going to be used as a model.
* Stalker_node => Basic node that use a given pipeline and given shape type to do object recognition. The model can be taken from either a file or a point Cloud topic.
* Stalker_node_nogui => Same without the GUI.
* Stalker_node_tld => Stalker node working in tandem with ros_open_tld. When open_tld found a model, it uses the bouding box to search for the object inside of it, drastically speeding up the process. If open_tld is lost, then it uses the full point cloud from the Kinect to search for the object. As sound as it has been found it sent a new bouding box to open_tld (still needs to be done !)






