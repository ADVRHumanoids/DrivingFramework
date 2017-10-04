#include <gtest/gtest.h>
#include <ros/package.h>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "mwoibn/robot_class/robot.h"

// Check if all compiled robot instances initializes propoerly
TEST(RobotClassTest, initializationBaseClass)
{


    std::string path = ros::package::getPath("tests_common");
    std::string file_name = path + "/resources/urdf/centauro.urdf";

    //RigidBodyDynamics::Model* _model_ptr = new RigidBodyDynamics::Model;

    //RigidBodyDynamics::Addons::URDFReadFromFile(file_name.c_str(), _model_ptr,
    //                                            false, false);

    EXPECT_NO_THROW(mwoibn::robot_class::Robot robot(file_name, "", true));


  // Failure case cannot run as it causes memory violation and consequently core
  // is aborted

//     file_name = "/centauro_urdf/test/centauro.urdf";

//     EXPECT_THROW(mwoibn::robot_class::Robot robot(file_name, true, true), std::invalid_argument);

}

//// Check all basic robot methods (get/set/update joint_state/velocity)
TEST(RobotClassTest, methodsBaseClass)
{
    bool is_static = true;

    std::string path = ros::package::getPath("tests_common");
    std::string file_name = path + "/resources/urdf/centauro.urdf";

    mwoibn::robot_class::Robot robot(file_name, "", true);

    mwoibn::VectorN joint_states =
        Eigen::VectorXd::Random(robot.getDofs());

    robot.state.set(joint_states, mwoibn::robot_class::INTERFACE::POSITION);
    robot.update();
    EXPECT_EQ(joint_states, robot.state.get(mwoibn::robot_class::INTERFACE::POSITION));

  //  robot.updateJointStates();
  //  EXPECT_EQ(joint_states, robot.state.get(mwoibn::robot_class::INTERFACE::POSITION));

    robot.state.set(joint_states, mwoibn::robot_class::INTERFACE::VELOCITY);
    EXPECT_EQ(joint_states, robot.state.get(mwoibn::robot_class::INTERFACE::VELOCITY));

 //   robot.updateJointVelocities();
 //   EXPECT_EQ(joint_states, robot.state.get(mwoibn::robot_class::INTERFACE::VELOCITY));

    EXPECT_EQ(is_static, robot.isStatic());

    EXPECT_FALSE(robot.controllers.send());

    robot.command.set(joint_states, mwoibn::robot_class::INTERFACE::POSITION);

    EXPECT_EQ(robot.command.get(mwoibn::robot_class::INTERFACE::POSITION), joint_states);

    EXPECT_EQ(robot.command.get(mwoibn::robot_class::INTERFACE::TORQUE), mwoibn::VectorN::Zero(robot.getDofs()));

    robot.command.set(joint_states, mwoibn::robot_class::INTERFACE::TORQUE);
    EXPECT_EQ(robot.command.get(mwoibn::robot_class::INTERFACE::TORQUE), joint_states);

    EXPECT_EQ(Eigen::VectorXi::Ones(robot.getDofs()), robot.getActuationState());

    is_static = false;
    mwoibn::robot_class::Robot robot_floating (file_name, "", true);
    mwobin::VectorInt test_actuation = mwobin::VectorInt::Ones(robot.getDofs());

    test_actuation.head(6) << 0,0,0, 0,0,0;
    EXPECT_EQ(test_actuation, robot_floating.getActuationState());

}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
