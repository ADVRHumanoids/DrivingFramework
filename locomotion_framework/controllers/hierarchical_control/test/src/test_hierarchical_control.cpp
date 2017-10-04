#include <gtest/gtest.h>
#include <ros/package.h>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "mwoibn/robot_class/robot.h"
#include "mwoibn/collision_model/robot_collision.h"
#include "mwoibn/hierarchical_control/hierarchical_controller.h"
#include "mwoibn/hierarchical_control/self_collision_task.h"
#include "mwoibn/hierarchical_control/cartesian_world_task.h"

#include <iostream>
#include <fstream>
#include <sstream>

//#define MAKE_LOG
#include "mwoibn/tests_common/test.h"
#include "mwoibn/tests_common/test_collision.h"


// Check if all compiled robot instances initializes propoerly
TEST(ControlerTaskTest, initializationBaseClass)
{
  mwoibn::hierarchical_control::HierarchicalController hc;
}


// Check all basic robot methods (get/set/update joint_state/velocity)
TEST(ControlerTaskTest, methodsBaseClass)
{

  std::string path;
  std::string file_name;

  std::unique_ptr<mwoibn::robot_class::Robot> robot_ptr = mwoibn::tests_common::_initTestRobot(true);
  mwoibn::robot_class::Robot& robot = *robot_ptr;
  std::unique_ptr<mwoibn::collision_model::RobotCollision> collision_ptr = mwoibn::tests_common::_initTestCollisionModel(robot);
  mwoibn::collision_model::RobotCollision& collision = *collision_ptr;

  mwoibn::hierarchical_control::HierarchicalController hc;

  mwoibn::hierarchical_control::SelfCollisionTask sc(collision, 0.05);

  mwoibn::Vector3 P_1;
  P_1 << 0.0, 0.0, -0.13;
  mwoibn::point_handling::PositionsHandler ik_1("pelvis", robot, {"arm1_7"}, {P_1});

  mwoibn::point_handling::PositionsHandler ik_2("pelvis", robot, {"arm2_7"}, {P_1});

  mwoibn::hierarchical_control::CartesianWorldTask ct_1(ik_1);

  mwoibn::hierarchical_control::CartesianWorldTask ct_2(ik_2);

  mwoibn::VectorN collision_states(robot.getDofs());

  collision_states <<  0.2,  0.0,  0.5, -1.2,  0.7, -0.5, 0.5,
                      -0.2,  0.3, -0.5,  1.0, -0.7,  0.5, 0.5,
                       0;

  mwoibn::VectorN non_collision_states(robot.getDofs());

  non_collision_states <<  0.2,  0.0, -0.5, -1.0,  0.7, -0.5, 0.5,
                          -0.2,  0.3,  0.5,  1.0, -0.7,  0.5, 0.5,
                           0;


  mwoibn::VectorN reference(3);
  reference <<  0.3, -0.5, 0;
  ct_1.setReference(reference);

  reference <<  0.3, 0.5, 0;
  ct_2.setReference(reference);

  mwoibn::VectorN gain(1);
/*
  robot.state.set(collision_states, mwoibn::robot_class::INTERFACE::POSITION);
  robot.update();

  gain << 1200;
  hc.addTask(&ct_1,gain);

  gain << 1000;
  hc.addTask(&ct_2,gain);

  gain << 900;
  hc.addTask(&sc,gain);

  hc.update();

  std::cerr << "CT_1 + CT_2 + SC" << std::endl;
  std::cerr << hc.getCommand() << std::endl;
*/

/*

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
  file_name = path + "/test/resources/hierarchical_controller.txt";
  std::ifstream myfile(file_name);

  if (!myfile.is_open())
    FAIL() << "couldn't open reference file";

  if (!mwoibn::tests_common::skipLine(&myfile, 1))
    FAIL() << "couldn't continue, file finished prematurely";
  Eigen::MatrixXd J_1 = mwoibn::tests_common::readMatrix(&myfile, 15, 1);

  if (!mwoibn::tests_common::skipLine(&myfile, 1))
    FAIL() << "couldn't continue, file finished prematurely";
  Eigen::MatrixXd J_2 = mwoibn::tests_common::readMatrix(&myfile, 15, 1);

  if (!mwoibn::tests_common::skipLine(&myfile, 1))
    FAIL() << "couldn't continue, file finished prematurely";
  Eigen::MatrixXd J_3 = mwoibn::tests_common::readMatrix(&myfile, 15, 1);

  if (!mwoibn::tests_common::skipLine(&myfile, 1))
    FAIL() << "couldn't continue, file finished prematurely";
  Eigen::MatrixXd J_4 = mwoibn::tests_common::readMatrix(&myfile, 15, 1);

  if (!mwoibn::tests_common::skipLine(&myfile, 1))
    FAIL() << "couldn't continue, file finished prematurely";
  Eigen::MatrixXd J_5 = mwoibn::tests_common::readMatrix(&myfile, 15, 1);
#endif

  robot.state.set(collision_states, mwoibn::robot_class::INTERFACE::POSITION);
  robot.update();

  gain << 1200;
  hc.addTask(&ct_1, gain);

  // CT_1
  hc.update();

#ifndef MAKE_LOG
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(hc.getCommand(), J_1, 0.1));
#endif
#ifdef MAKE_LOG
  std::cerr << "CT_1" << std::endl;
  std::cerr << hc.getCommand() << std::endl;
#endif
  gain << 900;
  hc.addTask(&sc, gain);



  // CT_1 + SC
  hc.update();

#ifndef MAKE_LOG
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(hc.getCommand(), J_3, 0.1));
#endif
#ifdef MAKE_LOG
  std::cerr << "CT_1 + SC" << std::endl;
  std::cerr << hc.getCommand() << std::endl;
#endif
  gain << 1000;
  hc.addTask(&ct_2, gain, 1);



  // CT_1 + CT_2 + SC
  hc.update();
#ifndef MAKE_LOG
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(hc.getCommand(), J_5, 0.1));
#endif
#ifdef MAKE_LOG
  std::cerr << "CT_1 + CT_2 + SC" << std::endl;
  std::cerr << hc.getCommand() << std::endl;
#endif
  hc.removeTask(0);

  // CT_2 + SC
  hc.update();
#ifndef MAKE_LOG
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(hc.getCommand(), J_4, 0.1));
#endif
#ifdef MAKE_LOG
  std::cerr << "CT_2 + SC" << std::endl;
  std::cerr << hc.getCommand() << std::endl;
#endif
  hc.removeTask(1);

  // CT_2
  hc.update();
#ifndef MAKE_LOG
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(hc.getCommand(), J_2, 0.1));
#endif
#ifdef MAKE_LOG
  std::cerr << "CT_2" << std::endl;
  std::cerr << hc.getCommand() << std::endl;
#endif
  gain << 900;
  hc.addTask(&sc, gain);

  // CT_2 + SC
  hc.update();
#ifndef MAKE_LOG
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(hc.getCommand(), J_4, 0.1));
#endif
  gain << 1200;
  hc.addTask(&ct_1, gain, 0);

  // CT_1 + CT_2 + SC
  hc.update();
#ifndef MAKE_LOG
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(hc.getCommand(), J_5, 0.1));
#endif
  hc.removeTask(1);

  // CT_1 + SC
  hc.update();
#ifndef MAKE_LOG
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(hc.getCommand(), J_3, 0.1));
#endif
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
