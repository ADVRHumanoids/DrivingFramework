# DrivingFramework

This repository contains the software framework designed as a tool for the locomotion research.  

The software provides a framework to quickly try out ideas by maximizing the flexibility and reconfigurability of the developed modules. It is acheived through the reconfigurable YAML files, and a simple end-user API.  An abstraction layers for the robot kinematics/dynamics are provided, and a communication layer that consists of the reconfigurable feedbacks/commands has been developed. This communication layer is automatically loaded based on the plugin initialization. The configurable feedbacks/commands allows, for example, to switch between the online-offline implementation of the plugin. Furthermore, the software provides a unified interface  for the non real-time ROS and the real-time XBotCore middlewares. Thanks to the automatically generated plugins, the software can be moved from the simulation to the hardware with one parameter change in the configuration file.

Furthermore, a Robot Points interface has been developed to simplify a definition of the locomotion problems by providing the common interface for the different robot characteristics and corresponding Jacobians. Examples of the components integrated with this interface include - among others - a rigid body point, point/body velocity, point/body acceleration, force/wrench,  center of mass and contact points. With these elements defined through the common interface a template class has been developed to menage the elements together while ensuring the access to individual points when necessary.

This repository contains implemented controllers and state estimators that have been in the simulation and in the experiments with the Centauro robot. It includes the centralized controller with the whole-body gravity compensation and gravity compensation for the series elastic actuators, an inverse-kinematics scheme, a hybrid legged-wheeled locomotion module, a joint-space controller as well as the simple state estimation and estimation of the ground reaction forces.    

# Warning
It has been developed to simplify the author PhD work, and it has not been designed as a package to be released. As a utilitarian tool developed with a very restricted time limit, no unit test has been developed, and the comments are scarcare to none.


# Third Party Software
* Eigen [http://eigen.tuxfamily.org/index.php?title=Main_Page], 
* RBDL [https://rbdl.bitbucket.io/],
* ros-control, ros-controllers [https://github.com/ros-controls], 
* srdfdom [https://github.com/ros-planning/srdfdom],
* ranges (a copy included in this repository) [https://github.com/ericniebler/range-v3],
* eiquadprog (a copy included in this repository),
* XBotCore (can be removed with the XBOT compiler flag - the packages.xml have to adjusted by hand).
