#include <gtest/gtest.h>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "mwoibn/robot_class/robot_ros.h"

// Check if all compiled robot instances initializes properly
TEST(DISABLED_RobotClassTest, initializationRos)
{
  try
  {

    mwoibn::robot_class::RobotRos robot_ros;
  }
  catch (const std::exception& exc)
  {
    ADD_FAILURE() << exc.what();
  }
  catch (...)
  {
    ADD_FAILURE() << "Unknown exception";
  }
  try
  {

    mwoibn::robot_class::RobotRos robot_ros("/robot_description");
  }
  catch (const std::exception& exc)
  {
    ADD_FAILURE() << exc.what();
  }
  catch (...)
  {
    ADD_FAILURE() << "Unknown exception";
  }
  try
  {

    mwoibn::robot_class::RobotRos robot_ros("/wrong_topic");
  }
  catch (...)
  {
    SUCCEED() << "Uncaught exception";
  }
}

// Check all basic robot methods (get/set/update joint_state/velocity)
TEST(DISABLED_RobotClassTest, methodsRos)
{
  try
  {
    bool is_static = false;
    mwoibn::robot_class::RobotRos robot("/robot_description", "", false, false);

    mwoibn::VectorN joint_states =
        Eigen::VectorXd::Random(robot.getDofs());

    robot.state.position.set(joint_states);
//    robot.update();
    EXPECT_EQ(joint_states, robot.state.position.get());

    robot.state.velocity.set(joint_states);
    EXPECT_EQ(joint_states, robot.state.velocity.get());


    EXPECT_EQ(is_static, robot.isStatic());

    EXPECT_FALSE(robot.controllers.send());


    robot.command.position.set(joint_states);
  }
  catch (...)
  {
    ADD_FAILURE() << "Uncaught exception";
  }
}
