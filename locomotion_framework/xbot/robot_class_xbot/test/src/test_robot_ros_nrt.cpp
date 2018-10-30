#include <gtest/gtest.h>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "mwoibn/robot_class/robot_ros_nrt.h"
#include "mwoibn/tests_common/test.h"

// Check if all compiled robot instances initializes propoerly
TEST(RobotClassTest, initializationRosNRT)
{
  try
  {

    mwoibn::robot_class::RobotRosNRT robot;
  }
  catch (...)
  {
    ADD_FAILURE() << "Uncaught exception";
  }
  try
  {

    mwoibn::robot_class::RobotRosNRT robot("/robot_description");
  }
  catch (...)
  {
    ADD_FAILURE() << "Uncaught exception";
  }
  try
  {

    mwoibn::robot_class::RobotRosNRT robot("/wrong_topic");
  }
  catch (...)
  {
    SUCCEED() << "Uncaught exception";
  }
}

// Check all basic robot methods (get/set/update joint_state/velocity)
TEST(RobotClassTest, methodsRosNRT)
{
  try
  {
    bool is_static = false;
    mwoibn::robot_class::RobotRosNRT robot("/robot_description", "", false, false, "joint_states", "/gazebo/link_states", false, true);

    mwoibn::VectorN standing_position(27);

    standing_position <<  0,
                         0.2,  0.3,  0.5, -1.2,  0.7, -0.5,  0.5,
                         -0.2,  0.3, -0.5,  1.0, -0.7,  0.5,  0.5,
                          0.8,  0.8, -0.8, -0.8,
                          0.9,  0.9,  0.9,  0.9,
                         -0.9, -0.9, -0.9, -0.9;

    mwoibn::VectorN joint_states = Eigen::VectorXd::Zero(robot.getDofs());
    robot.controllers.controller(0).mapFromController(standing_position, joint_states);

    mwoibn::VectorN orignal_states = robot.state.position.get();
    EXPECT_EQ(is_static, robot.isStatic());

    // robot set_command
    robot.command.position.set(joint_states);

    EXPECT_TRUE(robot.controllers.send());

    ros::Duration(2.0).sleep();
;
    ros::spinOnce();

    // checks if callback is working (if joint states changed any how, even if
    // controller is not working states should be different due to the numerical
    // errors in the simulation)
    EXPECT_FALSE(robot.state.position.get().norm() == orignal_states.norm());

    // check if both controller and mapping are working properly
    EXPECT_NEAR(joint_states.tail(robot.getDofs()-6).norm(), robot.state.position.get().tail(robot.getDofs()-6).norm(), 0.1);
    EXPECT_NEAR(joint_states[6], robot.state.position.get()[6], 0.1);
    EXPECT_NEAR(joint_states[7], robot.state.position.get()[7], 0.01);
    EXPECT_NEAR(joint_states[robot.getDofs() - 1],
                robot.state.position.get()[robot.getDofs() - 1], 0.01);
  }
  catch (...)
  {
    ADD_FAILURE() << "Uncaught exception";
  }
}

TEST(RobotClassTest, feedForwardSupport)
{
  double eps = 0.01;

  try
  {
    bool is_static = false;
    mwoibn::robot_class::RobotRosNRT robot;
    int i = 0;
    mwoibn::VectorN standing_position(27);
    mwoibn::VectorN feed_forward(27);

    standing_position <<  0,
                         0.2,  0.3,  0.5, -1.2,  0.7, -0.5,  0.5,
                         -0.2,  0.3, -0.5,  1.0, -0.7,  0.5,  0.5,
                          0.8,  0.8, -0.8, -0.8,
                          0.9,  0.9,  0.9,  0.9,
                         -0.9, -0.9, -0.9, -0.9;

    int dofs = robot.getDofs();

    mwoibn::VectorN joint_states = Eigen::VectorXd::Zero(dofs);
    mwoibn::VectorN feed_forward_controller = Eigen::VectorXd::Zero(dofs);
    mwoibn::VectorN full_state(2*dofs);

    robot.controllers.controller(0).mapFromController(standing_position, joint_states);

    EXPECT_EQ(is_static, robot.isStatic());

    // robot set_command
    robot.command.position.set(joint_states);

    EXPECT_TRUE(robot.controllers.send());

    ros::Duration(2.0).sleep();
;
    ros::spinOnce();

    mwoibn::VectorN orignal_states = robot.state.position.get();

//    EXPECT_FALSE(robot.controllers.controller(0).isFeedForward());


    // ACTIVATE FEED FORWARD
//    robot.activateFeedForward();

//    EXPECT_TRUE(robot.controllers.controller(0).isFeedForward());

    // CHECK SENDING ONLY POSITIONS
    robot.command.position.set(joint_states);

    EXPECT_TRUE(robot.controllers.send());

    ros::Duration(2.0).sleep();
;
    ros::spinOnce();

    EXPECT_TRUE(mwoibn::tests_common::compareMatrices(robot.state.position.get().tail(14), orignal_states.tail(14), eps));

    // SEND FULL STATE
    feed_forward.head(15) = 200*mwoibn::VectorN::Ones(15);
    feed_forward.tail(12) = mwoibn::VectorN::Zero(12);

    robot.controllers.controller(0).mapFromController(feed_forward, feed_forward_controller);

    full_state.head(dofs) = joint_states;
    full_state.tail(dofs) = feed_forward_controller;

    // SEND ONLY FEED FORWARD THERM
    mwoibn::VectorN zero = mwoibn::VectorN::Zero(dofs);
    robot.command.torque.set(zero);

    EXPECT_TRUE(robot.controllers.send());

    ros::spinOnce();

    ros::Duration(2.0).sleep();
;
    ros::spinOnce();

    EXPECT_TRUE(mwoibn::tests_common::compareMatrices(robot.state.position.get().tail(14), orignal_states.tail(14), eps));

    robot.command.position.set(zero);
    EXPECT_TRUE(robot.controllers.send());

    ros::spinOnce();

    ros::Duration(2.0).sleep();
;
    ros::spinOnce();

    EXPECT_FALSE(mwoibn::tests_common::compareMatrices(robot.state.position.get().tail(14), orignal_states.tail(14), 10*eps));

    robot.command.position.set(joint_states);
    EXPECT_TRUE(robot.controllers.send());

    ros::spinOnce();

    ros::Duration(2.0).sleep();
;
    ros::spinOnce();

    EXPECT_TRUE(mwoibn::tests_common::compareMatrices(robot.state.position.get().tail(14), orignal_states.tail(14), eps));

    robot.command.torque.set(feed_forward_controller);

    EXPECT_TRUE(robot.controllers.send());

    ros::spinOnce();

    ros::Duration(2.0).sleep();
;
    ros::spinOnce();

    EXPECT_FALSE(mwoibn::tests_common::compareMatrices(robot.state.position.get().tail(25), orignal_states.tail(25), 10*eps));
    std::cout << "robot.state.position.get()" << std::endl;
    std::cout << robot.state.position.get() << std::endl;

    std::cout << "original_states" << std::endl;
    std::cout << orignal_states << std::endl;

//    robot.deactivateFeedForward();

  }
  catch (...)
  {
    ADD_FAILURE() << "Uncaught exception";
  }
}
int main(int argc, char** argv)
{

  ros::init(argc, argv, "robot_class_test"); // initalize node

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
