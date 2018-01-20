#include <mgnss/odometry/odometry.h>
#include <config.h>

#include <mwoibn/loaders/robot.h>

#include <mwoibn/robot_class/robot_ros_nrt.h>
#include <mwoibn/robot_class/robot_xbot_nrt.h>

int main(int argc, char** argv)
{

  ros::init(argc, argv, "odometry"); // initalize node needed for the service

  ros::NodeHandle n;
  ros::Publisher pub;

  pub = n.advertise<custom_messages::CustomCmnd>("odometry", 1);

  std::string path = std::string(DRIVING_FRAMEWORK_WORKSPACE);
  mwoibn::loaders::Robot loader;
  mwoibn::robot_class::Robot& robot = loader.init(
      path + "DrivingFramework/locomotion_framework/configs/mwoibn_v2.yaml",
      "odometry",
      path + "DrivingFramework/locomotion_framework/configs/lower_body.yaml");

  mgnss::odometry::Odometry odometry(
      robot, {"wheel_1", "wheel_2", "wheel_3", "wheel_4"}, 0.078);

  odometry.init();
  while (ros::ok())
  {
    odometry.update();
    //    robot.command.set(command, mwoibn::robot_class::INTERFACE::POSITION);
    //    robot.command.set(velocities,
    //    mwoibn::robot_class::INTERFACE::VELOCITY);
    robot.update();
  }
}
