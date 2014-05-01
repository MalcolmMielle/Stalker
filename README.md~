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

Globally, the Stalker is ensemble of three class : 

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

Like this, _Pipeline_ receive only __Shapes__
