#include "mwoibn/hierarchical_control/tasks/center_of_mass_task.h"
#include <iostream>
#include <fstream>
#include <sstream>

#include "mwoibn/tests_common/test.h"
//#define MAKE_LOG


// Check if all compiled robot instances initializes propoerly
TEST(CenterOfMassTask, initialization)
{
  std::unique_ptr<mwoibn::robot_class::Robot> robot_ptr = mwoibn::tests_common::_initTestRobot(true);

  mwoibn::robot_class::Robot& robot = *robot_ptr;

  mwoibn::hierarchical_control::tasks::CenterOfMass com_task(robot);

}

// Check all basic robot methods (get/set/update joint_state/velocity)
TEST(ControlerTaskTest, methodsBaseClass)
{
  std::unique_ptr<mwoibn::robot_class::Robot> robot_ptr = mwoibn::tests_common::_initTestRobot(true);
  mwoibn::robot_class::Robot& robot = *robot_ptr;

  mwoibn::Vector3 P_2 = mwoibn::Vector3::Zero(3);
  P_2 << 0.0, 0.0, -0.13;

  mwoibn::hierarchical_control::tasks::CenterOfMass task(robot);

  EXPECT_EQ(task.getTaskSize(),2);
  EXPECT_EQ(task.getTaskDofs(),15);

  mwoibn::VectorN collision_states(robot.getDofs());

  collision_states <<  0.2,  0.0,  0.5, -1.2,  0.7, -0.5, 0.5,
                      -0.2,  0.3, -0.5,  1.0, -0.7,  0.5, 0.5,
                       0;

  mwoibn::VectorN non_collision_states(robot.getDofs());

  non_collision_states <<  0.2,  0.0, -0.5, -1.0,  0.7, -0.5, 0.5,
                          -0.2,  0.3,  0.5,  1.0, -0.7,  0.5, 0.5,
                           0;


  mwoibn::VectorN reference(2);

  reference <<  0.3, -0.5;

  task.setReference(reference);
  EXPECT_EQ(task.getReference(), reference);

#ifndef MAKE_LOG
  // Read reference file
  std::string path = ros::package::getPath("hierarchical_control");
  std::string file_name = path + "/test/resources/center_of_mass_task.txt";
  std::ifstream myfile(file_name);

  if (!myfile.is_open())
    FAIL() << "couldn't open reference file";

  if (!mwoibn::tests_common::skipLine(&myfile, 1))
    FAIL() << "couldn't continue, file finished prematurely";
  Eigen::MatrixXd J_1 = mwoibn::tests_common::readMatrix(&myfile, 2, 15);

  if (!mwoibn::tests_common::skipLine(&myfile, 1))
    FAIL() << "couldn't continue, file finished prematurely";
  Eigen::VectorXd e_1 = mwoibn::tests_common::readMatrix(&myfile, 2, 1);

  if (!mwoibn::tests_common::skipLine(&myfile, 1))
    FAIL() << "couldn't continue, file finished prematurely";
  Eigen::MatrixXd J_2 = mwoibn::tests_common::readMatrix(&myfile, 2, 15);

  if (!mwoibn::tests_common::skipLine(&myfile, 1))
    FAIL() << "couldn't continue, file finished prematurely";
  Eigen::VectorXd e_2 = mwoibn::tests_common::readMatrix(&myfile, 2, 1);
#endif


  robot.state.position.set(collision_states);
  robot.update();
  task.updateJacobian();
  task.updateError();
  task.update();

#ifndef MAKE_LOG
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(task.getJacobian(), J_1, 0.0001));
  EXPECT_FALSE(mwoibn::tests_common::compareMatrices(task.getPreviousJacobian(), J_1, 0.0001));
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(task.getError(), e_1, 0.0001));
  EXPECT_FALSE(mwoibn::tests_common::compareMatrices(task.getPreviousError(), e_1, 0.0001));
#endif

  task.update();

#ifndef MAKE_LOG
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(task.getJacobian(), J_1, 0.0001));
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(task.getPreviousJacobian(), J_1, 0.0001));
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(task.getError(), e_1, 0.0001));
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(task.getPreviousError(), e_1, 0.0001));
#endif

  robot.state.position.set(non_collision_states);
  robot.update();
  task.update();

#ifndef MAKE_LOG
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(task.getJacobian(), J_2, 0.0001));
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(task.getPreviousJacobian(), J_1, 0.0001));
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(task.getError(), e_2, 0.0001));
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(task.getPreviousError(), e_1, 0.0001));
#endif

#ifdef MAKE_LOG
  std::cerr << "collision Jacobian" << std::endl;
  std::cerr << task.getPreviousJacobian() << std::endl;
  std::cerr << "collision Error" << std::endl;
  std::cerr << task.getPreviousError() << std::endl;
  std::cerr << "non_collision Jacobian" << std::endl;
  std::cerr << task.getJacobian() << std::endl;
  std::cerr << "non_collision Error" << std::endl;
  std::cerr << task.getError() << std::endl;
#endif
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
