#include <gtest/gtest.h>
#include <ros/package.h>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "mwoibn/robot_class/robot.h"
#include "mwoibn/robot_class/robot_ros.h"

#include "mwoibn/collision_model/robot_collision_ros.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include "mwoibn/tests_common/test.h"

// Check if all compiled robot instances initializes propoerly
TEST(CollisionRosTest, initializationBaseClass)
{

  std::unique_ptr<mwoibn::robot_class::Robot> robot_ptr = mwoibn::tests_common::_initTestRobot(true);
  mwoibn::robot_class::Robot& robot = *robot_ptr;

  EXPECT_NO_THROW(mwoibn::collision_model::RobotCollisionRos(
      robot, "/robot_description", "/robot_semantic_description", "centauro"));

  EXPECT_NO_THROW(mwoibn::collision_model::RobotCollisionRos(
      robot, "/robot_description", "", "centauro"));

  EXPECT_THROW(mwoibn::collision_model::RobotCollisionRos(robot, "not_rigth",
                                                  "/robot_semantic_description",
                                                  "centauro"),
               std::invalid_argument);

  EXPECT_THROW(mwoibn::collision_model::RobotCollisionRos(robot, "/robot_description",
                                                  "/robot_semantic_description",
                                                  "not_here"),
               std::invalid_argument);
}

// Check all basic robot methods (get/set/update joint_state/velocity)
TEST(CollisionRosTest, methodsBaseClass)
{

  mwoibn::robot_class::RobotRos robot("/robot_description", "", false, false);
  mwoibn::collision_model::RobotCollisionRos collision_model(
      robot, "/robot_description", "/robot_semantic_description", "centauro");

  mwoibn::VectorN collision_states(robot.getDofs());

  collision_states << 0.2, 0.3, 0.5, -1.2, 0.7, -0.5, 0.5, -0.2, 0.3, -0.5, 1.0,
      -0.7, 0.5, 0.5, 0;

  mwoibn::VectorN non_collision_states(robot.getDofs());

  non_collision_states << 0.2, 0.3, -0.5, -1.0, 0.7, -0.5, 0.5, -0.2, 0.3, 0.5,
      1.0, -0.7, 0.5, 0.5, 0;

  robot.state.set(collision_states, mwoibn::robot_class::INTERFACE::POSITION);
  robot.update();
  // check if proper amount of collisions has been detected
  EXPECT_EQ(collision_model.updateCollisions(), 3);

  mwoibn::VectorN distances = collision_model.getDistances();

  // make sure the collisions has been updated at all
  EXPECT_NE(distances[2], 0);
  EXPECT_NE(distances[5], 0);

  // check if collisions are detected in expected places
  EXPECT_EQ(distances[18], 0);
  EXPECT_EQ(distances[25], 0);
  EXPECT_EQ(distances[26], 0);

  robot.state.set(non_collision_states, mwoibn::robot_class::INTERFACE::POSITION);
  robot.update();
  // check if proper amount of collisions has been detected
  EXPECT_EQ(collision_model.updateCollisions(), 0);

  distances = collision_model.getDistances();
  // check if previously detected collisions have been reset
  EXPECT_NE(distances[18], 0);
  EXPECT_NE(distances[25], 0);
  EXPECT_NE(distances[26], 0);

  // update again to the collision state
  robot.state.set(collision_states, mwoibn::robot_class::INTERFACE::POSITION);
  robot.update();
  // expect that it will update position for a previous set up
  EXPECT_FALSE(collision_model.updateCollision(18));

  mwoibn::collision_model::RobotCollision::Pair test_pair = collision_model.getPair(18);
  mwoibn::collision_model::RobotCollision::Object test_object_1 =
      collision_model.getObject(test_pair.link_1);

  EXPECT_EQ(test_object_1.link, "arm1_4");

  collision_model.updatePosition(test_pair.link_1);
  collision_model.updatePosition(test_pair.link_2);
  EXPECT_TRUE(collision_model.updateCollision(18));

  EXPECT_FALSE(collision_model.updateCollision(25));
  EXPECT_FALSE(collision_model.updateCollision(26));

  collision_model.updatePositions();
  EXPECT_TRUE(collision_model.updateCollision(25));
  EXPECT_TRUE(collision_model.updateCollision(26));

  // Test final methods - simple get methods
  EXPECT_EQ(collision_model.getObjectsNumber(), 15);
  EXPECT_EQ(collision_model.getPairsNumber(), 52);

  // Test final methods - parsers from robot model
  EXPECT_EQ(collision_model.getRobotDofs(), 15);
  EXPECT_EQ(collision_model.getJointStates(), collision_states);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_model_test"); // initalize node

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
