#include <gtest/gtest.h>
#include <ros/package.h>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "mwoibn/robot_class/robot.h"
#include "mwoibn/common/types.h"


// Check if all compiled robot instances initializes propoerly
TEST(ContactsTest, initializationBaseClass)
{
    std::string path = ros::package::getPath("tests_common");
    std::string file_name = path + "/resources/urdf/centauro_full_test.urdf";
    mwoibn::robot_class::Robot robot(file_name, "", true);

    std::string name = "front_left_leg";
    std::string body_name = "knee_1";
    mwoibn::Vector3 position;
    position << 0, 0, -0.10;
    Eigen::Matrix<double, 6, 6> constraints;
    constraints << 1, 0, 0, 0, 0, 0,
                   0, 1, 0, 0, 0, 0,
                   0, 0, 1, 0, 0, 0,
                   0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0;
    EXPECT_EQ(robot.contacts().add(position, body_name, false, constraints, name),0);

    name = "front_right_leg";
    body_name = "knee_2";
    EXPECT_EQ(robot.contacts().add(position, body_name, false, constraints, name),1);

    EXPECT_TRUE(robot.contacts().remove(0));
    EXPECT_FALSE(robot.contacts().remove(2));

    name = "front_left_leg";
    body_name = "knee_1";
    EXPECT_EQ(robot.contacts().add(position, body_name, false, constraints, name),1);

    EXPECT_EQ(robot.contacts().size(),2);
    EXPECT_EQ(0,robot.contacts().getId("front_right_leg"));
    EXPECT_EQ(1,robot.contacts().getId("front_left_leg"));
    EXPECT_EQ(-1,robot.contacts().getId("blabla"));

    EXPECT_EQ("front_left_leg",robot.contacts().contact(1).getName());

    EXPECT_EQ("",robot.contacts().contact(1).getName());

    mwoibn::robot_points::Contact contact= robot.contacts.contact(0);
    EXPECT_EQ("front_right_leg",contact.getName());

    EXPECT_THROW(robot.contacts().contact(2), std::out_of_range);

    EXPECT_EQ(0,robot.getActiveContactsNumber());

    robot.contacts().activate(0);
    EXPECT_EQ(1,robot.contacts().sizeActive());
    robot.activateContact(1);
    EXPECT_EQ(2,robot.contacts().sizeActive());

    std::vector<unsigned int> id = robot.contacts().sizeActive();
    EXPECT_EQ(0,id[0]);
    EXPECT_EQ(1,id[1]);

    std::vector<std::string> names;
    id = robot.contacts().getActive(&names);

    EXPECT_EQ("front_right_leg",names[0]);
    EXPECT_EQ("front_left_leg",names[1]);

}

//// Check all basic robot methods (get/set/update joint_state/velocity)
TEST(ContactsTest, methodsBaseClass)
{

    bool is_static = false;
    std::string path = ros::package::getPath("tests_common");
    std::string file_name = path + "/resources/urdf/centauro_full_test.urdf";

    mwoibn::robot_points::Robot robot(file_name, "", true);

    mwoibn::VectorN joint_states =
        Eigen::VectorXd::Random(robot.getDofs());

    robot.state.set(joint_states, mwoibn::robot_points::INTERFACE::POSITION);
    EXPECT_EQ(joint_states, robot.state.get(mwoibn::robot_points::INTERFACE::POSITION));

 //   robot.updateJointStates();
 //   EXPECT_EQ(joint_states, robot.state.get(mwoibn::robot_class::INTERFACE::POSITION));

    robot.state.set(joint_states, mwoibn::robot_class::INTERFACE::VELOCITY);
    EXPECT_EQ(joint_states, robot.state.get(mwoibn::robot_class::INTERFACE::VELOCITY));

 //   robot.updateJointVelocities();
 //   EXPECT_EQ(joint_states, robot.state.get(mwoibn::robot_class::INTERFACE::VELOCITY));

    EXPECT_EQ(is_static, robot.isStatic());

    EXPECT_FALSE(robot.controllers.send());

    robot.command.set(joint_states, mwoibn::robot_class::INTERFACE::POSITION);

}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
