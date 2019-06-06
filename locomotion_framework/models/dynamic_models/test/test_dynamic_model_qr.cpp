#include <gtest/gtest.h>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "mwoibn/robot_class/robot.h"
#include "mwoibn/dynamic_models/qr_decomposition.h"



TEST( DISABLED_QRTest, testInitializationAndMethods){


  std::string path = ros::package::getPath("tests_common");
  std::string file_name = path + "/resources/urdf/centauro.urdf";

  mwoibn::robot_class::Robot robot(file_name, "", false);

  mwoibn::VectorN joint_states = Eigen::VectorXd::Zero(robot.getDofs());


  joint_states <<  0,
                        0.2,  0.3,  0.5, -1.2,  0.7, -0.5,  0.5,
                       -0.2,  0.3, -0.5,  1.0, -0.7,  0.5,  0.5,
                        0.8,  0.8, -0.8, -0.8,
                        0.9,  0.9,  0.9,  0.9,
                       -0.9, -0.9, -0.9, -0.9;

  robot.state.position.set(joint_states);
  robot.update();

  for (int i = 0; i < robot.getContactsNumber(); i++)
    robot.activateContact(i);

  mwoibn::dynamic_models::QrDecomposition qr(robot);

  qr.update();

  // here comparison with preset values should be made
  // qr.getGravity();
  // here comparison with preset values should be made
  qr.getInertia();
  std::cerr << qr.getInertia() << std::endl;
  // here comparison with preset values should be made
  qr.getNonlinearEffects();

}
