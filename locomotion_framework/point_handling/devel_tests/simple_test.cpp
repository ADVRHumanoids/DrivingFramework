#include <iostream>
#include <ros/ros.h>
#include <string>
#include <rbdl/rbdl.h>
#include "point_handling/robot_points_handler.h"
#include "robot_class/robot_ros_nrt.h"

//#ifndef RBDL_BUILD_ADDON_URDFREADER
//	#error "Error: RBDL addon URDFReader not enabled."
//#endif

//#include <rbdl/addons/urdfreader/urdfreader.h>
using namespace RigidBodyDynamics;
//using namespace RigidBodyDynamics::Math;

int main (int argc, char* argv[]) {
  ros::init(argc, argv, "development_test"); // initalize node

  ros::NodeHandle node;

  // joint state class keeps joint states and provaides mapping between the rbdl and urdf 
  robot_class::RobotRosNRT joint_state("/robot_description", "", false, false);

// END: INITIALIZATION


// START: EXAMPLE
// Define the frames of the reference points
  std::vector<std::string> body_names = {"arm1_7", "arm2_7"};
  
// Define the position of the points defined in the previous line: 2 by 3 matrix, each line defines the position of one point  
  MatrixNd P = MatrixNd::Zero (2,3);
  P << 	0.0, 0.0, -0.15, 0.0, 0.0, -0.15;   

  std::vector<bool> orientation;
  orientation.push_back(true);
  orientation.push_back(false);
// Test for only one point
  point_handling::PointsHandler in = point_handling::PointsHandler(body_names, &joint_state, &P,   "torso_2", &orientation);
 
  ros::Rate loop_rate(10);
  MatrixNd J;

// Main loop
//  while (ros::ok()){

    in.getIKJacobian(&J, 0, true);
    std::cout << "IK jacobian\n" << J << std::endl;
    J = MatrixNd::Zero (6,joint_state.getDofs());
    in.getJacobian(&J, 0, true);
    std::cout << "getJacobian\n" << J << std::endl;
    J = MatrixNd::Zero (6,joint_state.getDofs());
    in.getIKJacobians(&J);
    std::cout << "getIKJacobians\n" << J << std::endl;   
    J = MatrixNd::Zero (9,joint_state.getDofs());
    in.getFullJacobian(&J);
    std::cout << "getFullJacobians\n" << J << std::endl;  

	VectorNd position = in.getCurrentPosition(0, false);
    std::cout << "current position\n" << position << std::endl; 
	
	position = in.getCurrentPosition(0, true);
    std::cout << "current position + orientation\n" << position << std::endl; 
	
	position = in.getCurrentPositions();
    std::cout << "whole state\n" << position << std::endl; 
	
	in.getInformation();
	
	in.checkPoint(0);

	in.checkPoint(1);

//  ros::spinOnce();
//    loop_rate.sleep();
//  }

  return 0;
}


