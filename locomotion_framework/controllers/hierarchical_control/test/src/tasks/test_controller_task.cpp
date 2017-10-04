#include <gtest/gtest.h>
#include <ros/package.h>

#include <rbdl/rbdl.h>

#include "mwoibn/hierarchical_control/controller_task.h"
#include <iostream>
#include <fstream>
#include <sstream>

// Check if all compiled robot instances initializes propoerly
TEST(ControlerTaskTest, initializationBaseClass)
{
  mwoibn::hierarchical_control::ControllerTask(8, 15);
}

// Check all basic robot methods (get/set/update joint_state/velocity)
TEST(ControlerTaskTest, methodsBaseClass)
{
  mwoibn::hierarchical_control::ControllerTask task(8, 15);

  EXPECT_EQ(task.getTaskSize(),8);
  EXPECT_EQ(task.getTaskDofs(),15);

  Eigen::MatrixXd jacobian_1 = 3*Eigen::MatrixXd::Random(8,15);
  Eigen::MatrixXd jacobian_2 = 5*Eigen::MatrixXd::Random(8,15);

  task.updateJacobian(jacobian_1);
  task.updateJacobian(jacobian_2);

  EXPECT_EQ(task.getJacobian(), jacobian_2);
  EXPECT_EQ(task.getPreviousJacobian(), jacobian_1);

  task.updateJacobian();

  EXPECT_EQ(task.getJacobian(), jacobian_2);
  EXPECT_EQ(task.getPreviousJacobian(), jacobian_1);

  Eigen::VectorXd error_1 = 3*Eigen::VectorXd::Random(8);
  Eigen::VectorXd error_2 = 5*Eigen::VectorXd::Random(8);

  task.updateError(error_1);
  task.updateError(error_2);

  EXPECT_EQ(task.getError(), error_2);
  EXPECT_EQ(task.getPreviousError(), error_1);

  task.updateError();

  EXPECT_EQ(task.getError(), error_2);
  EXPECT_EQ(task.getPreviousError(), error_1);

  task.update();

  EXPECT_EQ(task.getError(), error_2);
  EXPECT_EQ(task.getPreviousError(), error_1);
  EXPECT_EQ(task.getJacobian(), jacobian_2);
  EXPECT_EQ(task.getPreviousJacobian(), jacobian_1);

}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
