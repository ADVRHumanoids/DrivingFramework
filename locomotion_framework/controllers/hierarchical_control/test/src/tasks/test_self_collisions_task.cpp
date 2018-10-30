#include "mwoibn/hierarchical_control/tasks/self_collisions_task.h"
#include <iostream>
#include <fstream>
#include <sstream>

#include "mwoibn/tests_common/test.h"
#include "mwoibn/tests_common/test_collision.h"
//#define MAKE_LOG

// Check if all compiled robot instances initializes propoerly
TEST(ControlerTaskTest, initializationBaseClass)
{
  std::unique_ptr<mwoibn::robot_class::Robot> robot_ptr = mwoibn::tests_common::_initTestRobot(true);
  mwoibn::robot_class::Robot& robot = *robot_ptr;
  std::unique_ptr<mwoibn::collision_model::RobotCollision> collision_ptr =
      mwoibn::tests_common::_initTestCollisionModel(robot);
  mwoibn::collision_model::RobotCollision& collision = *collision_ptr;

  std::vector<double> safety_limits;

  for (int i = 0; i < collision.getPairsNumber(); i++)
    safety_limits.push_back(0.05);

  //  std::cout << mesh.getPairsNumber() << std::endl;
  safety_limits[3] = 0.002;
  safety_limits[45] = 0.002;

  EXPECT_NO_THROW(mwoibn::hierarchical_control::tasks::SelfCollisions task(
      collision, safety_limits));

  EXPECT_NO_THROW(
      mwoibn::hierarchical_control::tasks::SelfCollisions task(collision, {0.05}));

  EXPECT_NO_THROW(
      mwoibn::hierarchical_control::tasks::SelfCollisions task(collision, 0.05));

  EXPECT_THROW(mwoibn::hierarchical_control::tasks::SelfCollisions task(
                   collision, {0.05, 0.05}),
               std::invalid_argument);
}

// Check all basic robot methods (get/set/update joint_state/velocity)
 TEST(ControlerTaskTest, methodsBaseClass)
{
  std::string path;
  std::string file_name;
  // Try to load a model, if fail abort, as collision model depends on
  // robot_class
  std::unique_ptr<mwoibn::robot_class::Robot> robot_ptr = mwoibn::tests_common::_initTestRobot(true);
  mwoibn::robot_class::Robot& robot = *robot_ptr;
  std::unique_ptr<mwoibn::collision_model::RobotCollision> collision_ptr =
      mwoibn::tests_common::_initTestCollisionModel(robot);
  mwoibn::collision_model::RobotCollision& collision = *collision_ptr;


  mwoibn::hierarchical_control::tasks::SelfCollisions task(collision, 0.05);

  EXPECT_EQ(task.getTaskSize(),52);
  EXPECT_EQ(task.getTaskDofs(),15);

  mwoibn::VectorN collision_states(robot.getDofs());

  collision_states <<  0.2,  0.0,  0.5, -1.2,  0.7, -0.5, 0.5,
                      -0.2,  0.3, -0.5,  1.0, -0.7,  0.5, 0.5,
                       0;

  mwoibn::VectorN non_collision_states(robot.getDofs());

  non_collision_states <<  0.2,  0.0, -0.5, -1.0,  0.7, -0.5, 0.5,
                          -0.2,  0.3,  0.5,  1.0, -0.7,  0.5, 0.5,
                           0;
/*
  robot.state.position.set(non_collision_states);
  robot.update();

  task.updateError();
  task.updateJacobian();

  //task.update();
  std::cerr << "non_collision Jacobian" << std::endl;
  std::cerr << task.getJacobian() << std::endl;
  std::cerr << "non_collision Error" << std::endl;
  std::cerr << task.getError() << std::endl;
*/
#ifndef MAKE_LOG
  // Read reference file
  path = ros::package::getPath("hierarchical_control");
  file_name = path + "/test/resources/self_collisions_controller.txt";
  std::ifstream myfile(file_name);

  if (!myfile.is_open())
    FAIL() << "couldn't open reference file";

  if (!mwoibn::tests_common::skipLine(&myfile, 1))
    FAIL() << "couldn't continue, file finished prematurely";
  Eigen::MatrixXd J_1 = mwoibn::tests_common::readMatrix(&myfile, 52, 15);

  if (!mwoibn::tests_common::skipLine(&myfile, 1))
    FAIL() << "couldn't continue, file finished prematurely";
  Eigen::VectorXd e_1 = mwoibn::tests_common::readMatrix(&myfile, 52, 1);

  if (!mwoibn::tests_common::skipLine(&myfile, 1))
    FAIL() << "couldn't continue, file finished prematurely";
  Eigen::MatrixXd J_2 = mwoibn::tests_common::readMatrix(&myfile, 52, 15);

  if (!mwoibn::tests_common::skipLine(&myfile, 1))
    FAIL() << "couldn't continue, file finished prematurely";
  Eigen::VectorXd e_2 = mwoibn::tests_common::readMatrix(&myfile, 52, 1);
#endif

  robot.state.position.set(collision_states);
  robot.update();

  task.updateError();
  task.updateJacobian();

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
