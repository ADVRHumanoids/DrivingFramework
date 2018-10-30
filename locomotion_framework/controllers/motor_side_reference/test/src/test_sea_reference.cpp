#include "mwoibn/tests_common/test.h"
#include "mwoibn/motor_side_reference/sea_reference.h"
#include <memory>
#include "mwoibn/robot_class/robot_ros.h"
#include "mwoibn/gravity_compensation/simple_qr_gravity_compensation.h"

TEST(SEAReferenceTest, testFull){

//  std::unique_ptr<mwoibn::robot_class::Robot> robot_ptr = mwoibn::tests_common::_initTestRobot(true);
//  mwoibn::robot_class::Robot& robot = *robot_ptr;
  mwoibn::robot_class::RobotRos robot("/robot_description", "", true, true);
//  mwoibn::dynamic_models::QrDecomposition dynamic_model(robot); // online set up
//  mwoibn::gravity_compensation::SimpleQRGravityCompensation feed_forward(dynamic_model,
//                                                                 robot);
//  mwoibn::VectorN non_collision_states(robot.getDofs());

//  non_collision_states <<  0.2,  0.0, -0.5, -1.0,  0.7, -0.5, 0.5,
//                          -0.2,  0.3,  0.5,  1.0, -0.7,  0.5, 0.5,
//                           0;

//  robot.state.position.set(non_collision_states);
//  robot.update();


//  mwoibn::motor_side_reference::SeaReference reference(robot, feed_forward);

//  // some set position// positions should be predefined from srdf or something
//  reference.update();

//  reference.getCommand();

}
