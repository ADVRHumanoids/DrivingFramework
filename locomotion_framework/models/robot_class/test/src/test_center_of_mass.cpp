#include <rbdl/rbdl.h>
#include "mwoibn/robot_class/robot.h"
#include "mwoibn/robot_class/center_of_mass.h"

#include "mwoibn/test.h"

TEST(CenterOfMassTest, pureClass)
{
  // Again safe this data. and check comparing them

  std::string path = ros::package::getPath("tests_common");
  std::string file_name = path + "/resources/urdf/centauro_full_test.urdf";
  mwoibn::robot_class::Robot robot(file_name, "", true);

  mwoibn::robot_class::CenterOfMass com(robot.getModel(), robot.state.state(mwoibn::robot_class::INTERFACE::POSITION), robot.state.state(mwoibn::robot_class::INTERFACE::VELOCITY));

  com.compute();
//  std::cerr << "get" << std::endl;
//  std::cerr << com.getCOM() << std::endl;

  com.computeJacobian();
//  std::cerr << "getJacobian" << std::endl;
//  std::cerr << com.getJacobian() << std::endl;

  mwoibn::VectorN joint_states =
      Eigen::VectorXd::Zero(robot.getDofs());

  joint_states << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.8, 0.9, -0.9, 0.8, 0.9, -0.9, -0.8, 0.9, -0.9, -0.8, 0.9,
      -0.9, 0, 0.2, 0.3, 0.5, -1.2, 0.7, -0.5, 0.5, -0.2, 0.3, -0.5, 1.0, -0.7,
      0.5, 0.5,

  robot.state.set(joint_states, mwoibn::robot_class::INTERFACE::POSITION);
  robot.update();
  com.compute();
//  std::cerr << "getCom" << std::endl;
//  std::cerr << com.get() << std::endl;

  com.computeJacobian();
//  std::cerr << "getJacobian" << std::endl;
//  std::cerr << com.getJacobian() << std::endl;


}


TEST(CenterOfMassTest, robotWrapper)
{

  //this just checks that the computations from COM are the same as from the Robot (read previous log)
  std::string path = ros::package::getPath("tests_common");
  std::string file_name = path + "/resources/urdf/centauro_full_test.urdf";
  mwoibn::robot_class::Robot robot(file_name, "", true);

  robot.centerOfMass().update();
//  std::cerr << "robot getCom" << std::endl;
//  std::cerr << robot.getCenterOfMass() << std::endl;
//  std::cerr << "robot getJacobian" << std::endl;
//  std::cerr << robot.getCenterOfMassJacobian() << std::endl;

  mwoibn::VectorN joint_states =
      Eigen::VectorXd::Zero(robot.getDofs());

  joint_states << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.8, 0.9, -0.9, 0.8, 0.9, -0.9, -0.8, 0.9, -0.9, -0.8, 0.9,
      -0.9, 0, 0.2, 0.3, 0.5, -1.2, 0.7, -0.5, 0.5, -0.2, 0.3, -0.5, 1.0, -0.7,
      0.5, 0.5,

  robot.state.set(joint_states, mwoibn::robot_class::INTERFACE::POSITION);
  robot.update();
  robot.centerOfMass().update();
//  std::cerr << "getCom" << std::endl;
//  std::cerr << robot.getCenterOfMass() << std::endl;
//  std::cerr << "getJacobian" << std::endl;
//  std::cerr << robot.getCenterOfMassJacobian() << std::endl;


}
int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
