#include <gtest/gtest.h>
#include <ros/package.h>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "mwoibn/robot_class/robot.h"
#include "mwoibn/collision_model/robot_collision.h"

#include <iostream>
#include <fstream>
#include <sstream>

// Check if all compiled robot instances initializes propoerly
TEST(RobotClassTest, initializationBaseClass)
{

  std::string path;
  std::string file_name;

  path = ros::package::getPath("tests_common");
  file_name = path + "/resources/urdf/centauro.urdf";

  mwoibn::robot_class::Robot robot(file_name, "", true);

  // load test urdf file
  std::ifstream myurdf(file_name);
  if (!myurdf.is_open())
    FAIL() << "Couldn't read test urdf file, abort.";
  std::string urdf_file(static_cast<std::stringstream const&>(std::stringstream() << myurdf.rdbuf()).str());
  myurdf.close();


  // load test srdf file
  file_name = path + "/resources/srdf/centauro.srdf";
  std::string srdf_file = "";
  std::ifstream  mysrdf(file_name);
  if (!mysrdf.is_open())
    ADD_FAILURE() << "Couldn't read test srdf file, trying to continue. All srdf depentend tests will fail";
  else{
    srdf_file = static_cast<std::stringstream const&>(std::stringstream() << mysrdf.rdbuf()).str();
    mysrdf.close();
  }


  EXPECT_NO_THROW(mwoibn::collision_model::RobotCollision(robot, urdf_file, srdf_file, ros::package::getPath("centauro")));
  EXPECT_NO_THROW(mwoibn::collision_model::RobotCollision(robot, urdf_file, "", ros::package::getPath("centauro")));

  EXPECT_THROW( mwoibn::collision_model::RobotCollision(robot, "not_rigth", srdf_file, ros::package::getPath("centauro")),std::invalid_argument);
  EXPECT_THROW( mwoibn::collision_model::RobotCollision(robot, urdf_file, srdf_file, ros::package::getPath("centauro")+"not_here"),std::invalid_argument);

}

//// Check all basic robot methods (get/set/update joint_state/velocity)
TEST(CollisionModelTest, methodsBaseClass)
{
  std::string path;
  std::string file_name;


  path = ros::package::getPath("tests_common");
  file_name = path + "/resources/urdf/centauro.urdf";

  mwoibn::robot_class::Robot robot(file_name, "", true);

  // load test urdf file
  std::ifstream myurdf(file_name);
  if (!myurdf.is_open())
    FAIL() << "Couldn't read test urdf file, abort.";
  std::string urdf_file(static_cast<std::stringstream const&>(std::stringstream() << myurdf.rdbuf()).str());
  myurdf.close();


  // load test srdf file
  file_name = path + "/resources/srdf/centauro.srdf";
  std::string srdf_file = "";
  std::ifstream  mysrdf(file_name);
  if (!mysrdf.is_open())
    ADD_FAILURE() << "Couldn't read test srdf file, trying to continue. All srdf depentend tests will fail";
  else{
    srdf_file = static_cast<std::stringstream const&>(std::stringstream() << mysrdf.rdbuf()).str();
    mysrdf.close();
  }

  mwoibn::collision_model::RobotCollision collision_model(robot, urdf_file, srdf_file, ros::package::getPath("centauro"));


  mwoibn::VectorN collision_states(robot.getDofs());

  collision_states <<  0.2,  0.3,  0.5, -1.2,  0.7, -0.5, 0.5,
                      -0.2,  0.3, -0.5,  1.0, -0.7,  0.5, 0.5,
                       0;

  mwoibn::VectorN non_collision_states(robot.getDofs());

  non_collision_states <<  0.2,  0.3, -0.5, -1.0,  0.7, -0.5, 0.5,
                          -0.2,  0.3,  0.5,  1.0, -0.7,  0.5, 0.5,
                           0;

  robot.state.position.set(collision_states);
  robot.update();

  // check if proper amount of collisions has been detected
  EXPECT_EQ(collision_model.updateCollisions(),3);

  mwoibn::VectorN distances = collision_model.getDistances();

  // make sure the collisions has been updated at all
  EXPECT_NE(distances[2],0);
  EXPECT_NE(distances[5],0);

  // check if collisions are detected in expected places
  EXPECT_EQ(distances[18],0);
  EXPECT_EQ(distances[25],0);
  EXPECT_EQ(distances[26],0);

  robot.state.position.set(non_collision_states);
  robot.update();

  // check if proper amount of collisions has been detected
  EXPECT_EQ(collision_model.updateCollisions(),0);

  distances = collision_model.getDistances();
  // check if previously detected collisions have been reset
  EXPECT_NE(distances[18],0);
  EXPECT_NE(distances[25],0);
  EXPECT_NE(distances[26],0);

  // update again to the collision state
  robot.state.position.set(collision_states);
  robot.update();
  // expect that it will update position for a previous set up
  EXPECT_FALSE(collision_model.updateCollision(18));

  mwoibn::collision_model::RobotCollision::Pair test_pair = collision_model.getPair(18);
  mwoibn::collision_model::RobotCollision::Object test_object_1 = collision_model.getObject(test_pair.link_1);

  EXPECT_EQ(test_object_1.link, "arm1_4");

  collision_model.updatePosition(test_pair.link_1);
  collision_model.updatePosition(test_pair.link_2);
  EXPECT_TRUE(collision_model.updateCollision(18));

  EXPECT_FALSE(collision_model.updateCollision(25));
  EXPECT_FALSE(collision_model.updateCollision(26));

  collision_model.updatePositions();
  EXPECT_TRUE(collision_model.updateCollision(25));
  EXPECT_TRUE(collision_model.updateCollision(26));

  //Test final methods - simple get methods
  EXPECT_EQ(collision_model.getObjectsNumber(),15);
  EXPECT_EQ(collision_model.getPairsNumber(),52);

  //Test final methods - parsers from robot model
  EXPECT_EQ(collision_model.getRobotDofs(), 15);
  EXPECT_EQ(collision_model.getJointStates(), collision_states);

}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
