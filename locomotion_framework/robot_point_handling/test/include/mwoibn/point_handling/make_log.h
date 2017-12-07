#ifndef __MWOIBN_POINT_HANDLING_MAKE_LOG_H
#define __MWOIBN_POINT_HANDLING_MAKE_LOG_H

#include <iostream>
#include <ros/ros.h>
#include <string>
#include <rbdl/rbdl.h>
#include "mwoibn/point_handling/robot_points_handler.h"
#include "mwoibn/robot_class/robot.h"
#include "mwoibn/common/types.h"
#include <ros/package.h>

#ifndef RBDL_BUILD_ADDON_URDFREADER
	#error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/addons/urdfreader/urdfreader.h>
using namespace RigidBodyDynamics;
//using namespace RigidBodyDynamics::Math;



namespace mwoibn {

namespace robot_point_handling {

void makeLog() {
//  ros::init(argc, argv, "inverse_kinematics_1arm"); // initalize node

  std::string path = ros::package::getPath("centauro");
  std::string file_name = path+"/centauro_urdf/test/centauro.urdf";


  // joint state class keeps joint states and provaides mapping between the rbdl and urdf 
  mwoibn::robot_class::Robot robot(file_name, "", true);
  
  mwoibn::VectorN joint_states(robot.getDofs());

  joint_states << 0.2, 0.2, -0.2, 0.5, -0.7, 0.6, -0.9, 0.5, 0.2, -0.2, 0.5, 1.0, 0.6, 1.2, 0.5;

//  mwoibn::VectorN joint_states_moved(robot.getDofs());

//  joint_states_moved << -0.2, 0.3, 0.4, 0.5, -0.6, 0.3, -0.3, -0.5, 0.7, -0.2, 0.3, 0.5, -0.2, 1.0, 0.5;


  robot.state.set(joint_states, mwoibn::robot_class::INTERFACE::POSITION);
  robot.update();
// END: INITIALIZATION


// FIRST SET OF TESTS 
// Define the position of the points defined in the previous line: 2 by 3 matrix, each line defines the position of one point  
  mwoibn::Vector3 P_1;
  P_1 << 	0.0, 0.0, -0.15;

// Test for only one point
  mwoibn::point_handling::PositionsHandler position_handler("torso_2", robot, {"arm1_7", "arm2_7"}, {P_1, P_1});
  mwoibn::point_handling::OrientationsHandler orientation_handler("torso_2", robot, {"arm1_7", "arm2_7"}, {P_1, P_1});
  mwoibn::point_handling::FullStatesHandler full_state_handler("torso_2", robot, {"arm1_7", "arm2_7"}, {P_1, P_1});

  mwoibn::Matrix J;
    std::cerr << "Log data for points (0.0, 0.0, -0.15) of \"arm1_7\" and (0.0, 0.0, -0.15) \"arm2_7\" with respect to \"torso_2\"" << std::endl;
    // TEST JACOBIANS
    // check if jacobian of specific point is computed properly, try it for the multiple points to see if it works properly that way (add to class documentation if Matrix size is ensured or it has to have a correct size)
    // FIRST MATRIX
    J = full_state_handler.getReducedPointJacobian(1); // with orientation
    std::cerr << "IK jacobian with orientation\n" << J << std::endl;

    // SECOND MATRIX
    J = mwoibn::Matrix::Zero (6,robot.getDofs());
    full_state_handler.getFullPointJacobian(1, J);  // with orientation
    std::cerr << "getJacobian with orientation\n" << J << std::endl;

    // THIRD MATRIX
    J = mwoibn::Matrix::Zero (6,robot.getDofs());
    J = position_handler.getReducedJacobian();
    std::cerr << "getIKJacobians as defined in initialization\n" << J << std::endl;


    // FOURTH MATRIX
    J = mwoibn::Matrix::Zero (6,robot.getDofs());
    position_handler.getFullJacobian(J); // only position
    std::cerr << "getFullJacobians as defined in initialization\n" << J << std::endl;

    // FIFTH MATRIX
    mwoibn::VectorN position = orientation_handler.getPointStateWorld(0); // ponly position
    std::cerr << "current position of first point without orientation\n" << position << std::endl;

    // SIXTH MATRIX
    mwoibn::VectorN orientation(4);
    orientation << 0,0,0,1;
    std::cerr << "current position of first point + orientation\n" << orientation << "\n" << position << std::endl;

    // SEVENTH MATRIX
    position = position_handler.getFullStateWorld(); // all positions according to the original set-up
    std::cerr << "show all points as defined during initialization\n" << position << std::endl;
	


  // SECOND SET OF TESTS
  // check things for only one point
  std::cerr << "Log data for points (0.0, 0.0, -0.15) of \"arm1_7\" with respect to \"arm1_2\"" << std::endl;
  mwoibn::point_handling::PositionsHandler position_handler_2("arm1_2", robot, {"arm1_7"}, {P_1});
  mwoibn::point_handling::OrientationsHandler orientation_handler_2("arm1_2", robot, {"arm1_7"}, {P_1});
  mwoibn::point_handling::FullStatesHandler full_state_handler_2("arm1_2", robot, {"arm1_7"}, {P_1});

  mwoibn::Vector3 P_2 = mwoibn::Vector3::Zero (3);
  P_2 << 0.1, -0.2, -0.05;   

    // FIRST MATRIX
    J = mwoibn::Matrix::Zero (3,5);
    J = position_handler_2.getReducedPointJacobian(0);
    std::cerr << "IK jacobian\n" << J << std::endl;

    // SECOND MATRIX
    J = mwoibn::Matrix::Zero (3,robot.getDofs());
    position_handler_2.getFullPointJacobian(0, J);
    std::cerr << "getJacobian\n" << J << std::endl;

    // THIRD MATRIX
    J = mwoibn::Matrix::Zero (3,robot.getDofs());
    position_handler_2.getFullJacobian(J);
    std::cerr << "getFullJacobians\n" << J << std::endl;

    // FOURTH MATRIX
    mwoibn::VectorN position_2 = position_handler_2.getPointStateWorld(0);
    std::cerr << "current position\n" << position_2 << std::endl;
    std::cerr << "Change tracked point position to (0.1, -0.2, -0.05)" << std::endl;

    // FIFTH MATRIX
    position_handler_2.setPointStateFixed(0, P_2);
    position = position_handler_2.getPointStateWorld(0);
    std::cerr << "current position of new point\n" << position << std::endl;


    // SIXTH MATRIX
    position_handler_2.setPointStateWorld(0, position_2);
   	
    position = position_handler_2.getPointStateWorld(0);
    std::cerr << "whole state\n" << position << std::endl;
}

} // namespace package
} // namespace library

#endif
