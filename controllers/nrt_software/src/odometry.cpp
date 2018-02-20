#include <mgnss/odometry/odometry.h>
#include <config.h>

#include <mwoibn/loaders/robot.h>

#include <mwoibn/robot_class/robot_ros_nrt.h>
#include <mwoibn/robot_class/robot_xbot_nrt.h>

//// LOG
//#include <iostream>
//#include <fstream>
//#include <iomanip>
//#include <ctime>
//#include <chrono>
//// !LOG

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

  std::ostringstream oss;
  std::ofstream file;

  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  oss << "odometry_log_" << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << ".txt";

  file.open(oss.str(),  std::ios::out);
  file << "time,"
       << "raw_x,"      << "raw_y,"     << "raw_z,"
       << "raw_tx,"     << "raw_ty,"    << "raw_tz,"
       << "fil_x,"      << "fil_y,"     << "fil_z,"
       << "fil_tx,"     << "fil_ty,"    << "fil_tz,"
       << "\n";

  Eigen::IOFormat fmt(6, 0, ", ", ", ", "", "", "", "");
  file.flush();

  mwoibn::VectorN print(13);
  double start = ros::Time::now().toSec();
  double now = ros::Time::now().toSec();



  while (ros::ok())
  {
    odometry.update();
    //    robot.command.set(command, mwoibn::robot_class::INTERFACE::POSITION);
    //    robot.command.set(velocities,
    //    mwoibn::robot_class::INTERFACE::VELOCITY);
    now = ros::Time::now().toSec();
    print.setZero();
    print[0] =  now - start;
    print.segment<6>(1) =  odometry.getRaw();

    robot.update();

    print.segment<6>(7) =  odometry.getFiltered();

    file << print.transpose().format(fmt) << "\n";
  }

  file.flush();
  file.close();

}
