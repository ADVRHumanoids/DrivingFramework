#include <gtest/gtest.h>
#include <ros/package.h>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "mwoibn/robot_class/robot.h"

#include "mwoibn/test.h"
TEST(ActuatorsTest, initialization)
{

  std::string path = ros::package::getPath("tests_common");
  std::string file_name = path + "/resources/urdf/centauro_full_test.urdf";

  mwoibn::robot_class::Robot robot(file_name, "", true);

  mwoibn::robot_class::SeriesElasticActuator elasticActuator(1000, 0.2, 5000, 2,
                                                             "dsret");

  robot.actuators().add(std::unique_ptr<mwoibn::robot_class::Actuator>(
      new mwoibn::robot_class::Actuator(mwoibn::robot_class::RIGID, 1000, 0.2,
                                        "sfdljlfg")));
  robot.actuators().add(std::unique_ptr<mwoibn::robot_class::Actuator>(
      new mwoibn::robot_class::SeriesElasticActuator(1000, 0.2, 5000, 2,
                                                     "dsret")));
  robot.actuators().add(std::unique_ptr<mwoibn::robot_class::Actuator>(
      new mwoibn::robot_class::Actuator(mwoibn::robot_class::ACTUATOR_TYPE::UNACTUATED)));

  EXPECT_EQ(robot.actuators().size(), 3);

  mwoibn::VectorN state(3);
  state << 1, 2, 3;

  robot.actuators().setStates(state);

  EXPECT_TRUE(
      mwoibn::compareMatrices(robot.actuators().getStates(), state, 0.001));

  robot.actuators().setState(1, 4);

  state[1] = 4;
  EXPECT_TRUE(
      mwoibn::compareMatrices(robot.actuators().getStates(), state, 0.001));
  EXPECT_EQ(robot.actuators().getState(1), 4);

  state << 4, 5, 6;

  robot.actuators().setVelocities(state);

  EXPECT_TRUE(
      mwoibn::compareMatrices(robot.actuators().getVelocities(), state, 0.001));

  robot.actuators().setVelocity(1, 2);

  state[1] = 2;
  EXPECT_TRUE(
      mwoibn::compareMatrices(robot.actuators().getVelocities(), state, 0.001));
  EXPECT_EQ(robot.actuators().getVelocity(1), 2);

  Eigen::VectorXi types(3);
  types << mwoibn::robot_class::RIGID, mwoibn::robot_class::ACTUATOR_TYPE::ELASTIC,
      mwoibn::robot_class::ACTUATOR_TYPE::UNACTUATED;

  EXPECT_TRUE(mwoibn::compareMatrices(robot.actuators().getActuationTypes(),
                                      types, 0.001));

  EXPECT_TRUE(compareStdVectors(
      robot.actuators().getActuationTypes(mwoibn::robot_class::ACTUATOR_TYPE::ELASTIC),
      {false, true, false}, 0.001));

  EXPECT_EQ(robot.actuators().getActuatorName(0), "sfdljlfg");
  EXPECT_EQ(robot.actuators().getActuatorType(2),
            mwoibn::robot_class::ACTUATOR_TYPE::UNACTUATED);

  mwoibn::Matrix test_matrix =
      mwoibn::Matrix::Zero(3, 3);
  test_matrix(1, 1) = 5000;
  EXPECT_TRUE(mwoibn::compareMatrices(robot.actuators().getStiffnessMatrix(),
                                      test_matrix, 0.0001));
  test_matrix(1, 1) = 2;
  EXPECT_TRUE(mwoibn::compareMatrices(robot.actuators().getDampingMatrix(),
                                      test_matrix, 0.0001));
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
