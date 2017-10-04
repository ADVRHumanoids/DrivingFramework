#include <gtest/gtest.h>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "mwoibn/robot_class/robot_ros_nrt.h"
#include "mwoibn/common/types.h"


// Check methods releated to contact points
TEST(ContactsTest, methodsContacts)
{
  try
  {
    bool is_static = false;

    mwoibn::robot_class::RobotRosNRT robot("/robot_description", "", true, true, "joint_states", "/gazebo/link_states", true, true);

    ros::Duration(2.0).sleep();
    ros::spinOnce();

    mwoibn::VectorN standing_position(27);
    mwoibn::VectorN raise_leg(27);
    mwoibn::VectorN joint_states = Eigen::VectorXd::Zero(robot.getDofs());

    standing_position <<  0,
                          0.2,  0.3,  0.5, -1.2,  0.7, -0.5,  0.5,
                         -0.2,  0.3, -0.5,  1.0, -0.7,  0.5,  0.5,
                          0.8,  0.8, -0.8, -0.8,
                          0.9,  0.9,  0.9,  0.9,
                         -0.9, -0.9, -0.9, -0.9;

    raise_leg <<  0,
                  0.2,  0.3,  0.5, -1.2,  0.7, -0.5,  0.5,
                 -0.2,  0.3, -0.5,  1.0, -0.7,  0.5,  0.5,
                  0.8,  0.8, -0.8, -0.8,
                  1.5,  1.2,  1.2,  0.9,
                 -1.5, -1.2, -0.9, -0.9;

    standing_position[15] = 1.57;
    standing_position[16] = 1.57;

    robot.controllers.controller(0).mapFromController(standing_position, joint_states);
    robot.command.set(joint_states, mwoibn::robot_class::INTERFACE::POSITION);
    robot.controllers.send();
    ros::Duration(2.0).sleep();
    ros::spinOnce();

    EXPECT_EQ(4,robot.contacts().sizeActive());

    robot.contacts().contact(0).getPointJacobian();
    std::cerr << "Another constraint " << std::endl;
    robot.contacts().contact(1).getPointJacobian();

    robot.controllers.controller(0).mapFromController(raise_leg, joint_states);
    robot.command.set(joint_states, mwoibn::robot_class::INTERFACE::POSITION);
    robot.controllers.send();

    ros::Duration(2.0).sleep();
    ros::spinOnce();

    EXPECT_EQ(3, robot.contacts().sizeActive());

    std::vector<unsigned int> active = robot.contacts().getActive();
    if(robot.contacts().sizeActive() >= 3){
      EXPECT_EQ(0,active[0]);
      EXPECT_EQ(1,active[1]);
      EXPECT_EQ(3,active[2]);
    }
    EXPECT_EQ(3,active.size());
/*
    std::cerr << "robot.contacts().getJacobian()" << std::endl;

    std::cerr << robot.contacts().getJacobian() << std::endl;

   std::cerr << "robot.contacts().getMinimumJacobian()" << std::endl;

    std::cerr << robot.contacts().getMinimumJacobian() << std::endl;
    jacobians.clear();
    jacobians = robot.contacts().getJacobians();

    i_max = jacobians.size();
    i = 0;
    std::cerr << "jacobians.size()\t" << i_max << std::endl;

    for(auto& jacobian: jacobians){
    std::cerr << jacobians[i] << std::endl;
    i++;
    }

    jacobians.clear();
    jacobians = robot.contacts().getMinimumJacobians();
    i_max = jacobians.size();
    i = 0;
    std::cerr << "jacobians.size()\t" << i_max << std::endl;

    for(auto& jacobian: jacobians){
    std::cerr << jacobians[i] << std::endl;
    i++;
    }
*/
    robot.contacts().remove(0);

    ros::Duration(2.0).sleep();
    ros::spinOnce();

    EXPECT_EQ(2, robot.contacts().sizeActive());

    active = robot.contacts().getActive();
    if(robot.contacts().sizeActive() == 2){
    EXPECT_EQ(0,active[0]);
    EXPECT_EQ(2,active[1]);
    }
    EXPECT_EQ(2,active.size());



  }
  catch (...)
  {
    ADD_FAILURE() << "Uncaught exception";
  }
}


//TEST TO CHECK IF THE JACOBIAN IS COMPUTED CORECTLY COMPARING WITH ORIGINAL RBDL COMPUTATIONS
// The contactsTest is compared with respect to the results from RBDL constraints set and comparing with predefined values
TEST( ContactsTest, testJacobian){

  bool is_static = false;
  //The contacts callback has to be checked robot with controllers, therefore RobotRosNRT has to be launched
  std::unique_ptr<mwoibn::robot_class::RobotRosNRT> robot_ptr;
  ASSERT_NO_THROW(robot_ptr.reset(new mwoibn::robot_class::RobotRosNRT));

  mwoibn::robot_class::RobotRosNRT& robot = *robot_ptr;

  ros::Duration(2.0).sleep();
  ros::spinOnce();

  mwoibn::VectorN standing_position(27);
  mwoibn::VectorN joint_states = Eigen::VectorXd::Zero(robot.getDofs());

  standing_position <<  0,
                        0.2,  0.3,  0.5, -1.2,  0.7, -0.5,  0.5,
                       -0.2,  0.3, -0.5,  1.0, -0.7,  0.5,  0.5,
                        0.8,  0.8, -0.8, -0.8,
                        0.9,  0.9,  0.9,  0.9,
                       -0.9, -0.9, -0.9, -0.9;

  standing_position[15] = 1.57;
  standing_position[16] = 1.57;

  robot.controllers.controller(0).mapFromController(standing_position, joint_states);
  robot.command.set(joint_states, mwoibn::robot_class::INTERFACE::POSITION);
  robot.controllers.send();
  ros::Duration(2.0).sleep();
  ros::spinOnce();



  RigidBodyDynamics::ConstraintSet test_constraints;

  mwoibn::Vector3 normal;
  mwoibn::Vector3 point;

  point << 0, 0, -0.25;

  // FIRST LEG
  normal << 1,0,0;
  test_constraints.AddConstraint(robot.getModel().GetBodyId("knee_1"), point, normal, "front_left_leg_x");
  normal << 0.707106,0.707106,0;
  test_constraints.AddConstraint(robot.getModel().GetBodyId("knee_1"), point, normal, "front_left_leg_y");
  normal << 0,0,1;
  test_constraints.AddConstraint(robot.getModel().GetBodyId("knee_1"), point, normal, "front_left_leg_z");

  // SECOND LEG
  normal << 1,0,0;
  test_constraints.AddConstraint(robot.getModel().GetBodyId("knee_2"), point, normal, "front_right_leg_x");
  normal << 0,1,0;
  test_constraints.AddConstraint(robot.getModel().GetBodyId("knee_2"), point, normal, "front_right_leg_y");
  normal << 0,0,1;
  test_constraints.AddConstraint(robot.getModel().GetBodyId("knee_2"), point, normal, "front_right_leg_z");

  // THIRD LEG
  normal << 1,0,0;
  test_constraints.AddConstraint(robot.getModel().GetBodyId("knee_3"), point, normal, "rear_left_leg_x");
  normal << 0,1,0;
  test_constraints.AddConstraint(robot.getModel().GetBodyId("knee_3"), point, normal, "rear_left_leg_y");
  normal << 0,0,1;
  test_constraints.AddConstraint(robot.getModel().GetBodyId("knee_3"), point, normal, "rear_left_leg_z");

  // FOURTH LEG
  normal << 1,0,0;
  test_constraints.AddConstraint(robot.getModel().GetBodyId("knee_4"), point, normal, "rear_right_leg_x");
  normal << 0,1,0;
  test_constraints.AddConstraint(robot.getModel().GetBodyId("knee_4"), point, normal, "rear_right_leg_y");
  normal << 0,0,1;
  test_constraints.AddConstraint(robot.getModel().GetBodyId("knee_4"), point, normal, "rear_right_leg_z");

  test_constraints.Bind(robot.getModel());

  mwoibn::Matrix jacobian_rbdl(12, robot.getDofs());


  std::cerr << "RBDL JACOBIAN" << std::endl;
  robot.contacts().contact(0).activate();
  robot.contacts().contact(1).activate();
  robot.contacts().contact(2).activate();
  robot.contacts().contact(3).activate();
  EXPECT_EQ(4,robot.contacts().sizeActive());

  RigidBodyDynamics::CalcContactJacobian(robot.getModel(), robot.state.get(mwoibn::robot_class::INTERFACE::POSITION), test_constraints, jacobian_rbdl, true);

  std::cerr << jacobian_rbdl << std::endl;

  std::cerr << "CUSTOM JACOBIAN" << std::endl;

  std::cerr << robot.contacts().getJacobian() << std::endl;

}

/*
int main(int argc, char** argv)
{

  ros::init(argc, argv, "contacts_test"); // initalize node

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
*/
