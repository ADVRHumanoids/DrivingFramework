#include <gtest/gtest.h>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "mwoibn/robot_class/robot_ros_nrt.h"
#include "mwoibn/gravity_compensation/simple_qr_gravity_compensation.h"
#include "mwoibn/dynamic_models/qr_decomposition.h"
#include "mwoibn/tests_common/test.h"

using mwoibn::robot_class::INTERFACE;

TEST(QRTest, testFull)
{
  double eps = 0.01;

  bool is_static = false;
  // The contacts callback has to be checked robot with controllers, therefore
  // RobotRosNRT has to be launched
  mwoibn::robot_class::RobotRosNRT robot;

  mwoibn::dynamic_models::QrDecomposition dynamic_model(robot);
  std::unique_ptr<mwoibn::gravity_compensation::SimpleQRGravityCompensation>
      gravity_ptr;

  EXPECT_NO_THROW(gravity_ptr.reset(
      new mwoibn::gravity_compensation::SimpleQRGravityCompensation(dynamic_model)));

  mwoibn::gravity_compensation::SimpleQRGravityCompensation& gravity = *gravity_ptr;

  ros::Duration(1.0).sleep();
  ros::spinOnce();

  mwoibn::VectorN standing_position(27);
  mwoibn::VectorN joint_states =
      Eigen::VectorXd::Zero(robot.getDofs());

  standing_position << 0, 0.2, 0.3, 0.5, -1.2, 0.7, -0.5, 0.5, -0.2, 0.3, -0.5,
      1.0, -0.7, 0.5, 0.5, 0.8, 0.8, -0.8, -0.8, 0.9, 0.9, 0.9, 0.9, -0.9, -0.9,
      -0.9, -0.9;

  //  standing_position[15] = 1.57;
  //  standing_position[16] = 1.57;

  robot.controllers.controller(0).mapFromController(standing_position, joint_states);
  robot.command.position.set(joint_states);

  robot.controllers.send();

  ros::Duration(2.0).sleep();
  ros::spinOnce();

  mwoibn::VectorN orginal_states = robot.state.position.get();

  mwoibn::VectorN oldGravity = gravity.getCommand();

  gravity.update();

  EXPECT_FALSE(mwoibn::tests_common::compareMatrices(oldGravity.tail(14),
                               gravity.getCommand().tail(14), 10 * eps));

  // gravity.setCommand();

  robot.controllers.send();
  ros::Duration(2.0).sleep();
  ros::spinOnce();

  EXPECT_FALSE(mwoibn::tests_common::compareMatrices(
      robot.commad.torque.get,
      mwoibn::VectorN::Zero(robot.getDofs()), 10 * eps));

  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(robot.state.position.get().tail(14),
                              orginal_states.tail(14), 10 * eps));
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "robot_class_test"); // initalize node

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
