#include <mwoibn/robot_class/robot_xbot_nrt.h>
#include <mwoibn/robot_class/robot_ros_nrt.h>
//#include <mwoibn/robot_class/wheel_contact.h>

#include <mwoibn/hierarchical_control/hierarchical_controller.h>
#include <mwoibn/hierarchical_control/joint_positions_task.h>
#include <mwoibn/hierarchical_control/center_of_mass_task.h>
#include <mwoibn/hierarchical_control/constraints_task.h>
#include <mwoibn/hierarchical_control/controller_task.h>
#include <mwoibn/hierarchical_control/cartesian_selective_task.h>
#include <mwoibn/hierarchical_control/orientation_world_task.h>
#include <mwoibn/hierarchical_control/orientation_selective_task.h>

//#ifdef VISUALIZATION_TOOLS
#include <mwoibn/visualization_tools/rviz_track_point.h>

#include <mwoibn/point_handling/robot_points_handler.h>
//#endif
#include <custom_services/updateGains.h>
#include <custom_services/updatePrint.h>
#include <custom_messages/CustomCmnd.h>

// trajectory generation
//#include <mwoibn/reference_generation/line.h>
//#include <mwoibn/reference_generation/local_circle.h>
//#include <mgnss/communication/basic_handler.h>

// temp
//#include <MathGeoLib/Algorithm/Random/LCG.h>
//#include <MathGeoLib/Geometry/GeometryAll.h>

#include <mwoibn/eigen_utils/eigen_utils.h>

bool wheelHandler(custom_services::updateGains::Request& req,
                  custom_services::updateGains::Response& res,
                  double* x, double *y, double *z, double* th);

int main(int argc, char** argv)
{

  ros::init(argc, argv, "gravity_test"); // initalize node needed for the service

  ros::NodeHandle n;

  mwoibn::robot_class::RobotXBotNRT robot("/home/centauro/src/catkin_malgorzata/src/DrivingFramework/locomotion_framework/configs/mwoibn_v2.yaml" ,"higher_scheme");

  robot.update();

  RigidBodyDynamics::Math::VectorNd command =
      Eigen::VectorXd::Zero(robot.getDofs());

  mwoibn::hierarchical_control::ConstraintsTask constraints_task(robot);

  Eigen::Vector3d pelvis;
  pelvis << 1, 1, 1;

  mwoibn::point_handling::PositionsHandler pelvis_ph("ROOT", robot, {"pelvis"});
  mwoibn::hierarchical_control::CartesianSelectiveTask pelvis_hight(
      pelvis_ph,  pelvis);

  RigidBodyDynamics::Math::Vector3d axis;
  axis << 1, 1, 1;
  mwoibn::point_handling::OrientationsHandler pelvis_orn("ROOT", robot, {"pelvis"});
  mwoibn::hierarchical_control::OrientationSelectiveTask pelvis_orientation(pelvis_orn, axis, robot);

  axis << 0, 0, 1;
  pelvis_orientation.setReference(
      0, pelvis_orientation.getOffset(0)*mwoibn::Quaternion::fromAxisAngle(axis, 0.0));
//  pelvis_orientation.setReference(0, RigidBodyDynamics::Math::Quaternion(0,0,0,1));
  mwoibn::hierarchical_control::HierarchicalController hierarchical_controller;

  Eigen::Vector3d com_point = pelvis_ph.getPointStateWorld(0);
  double z = com_point[2]*robot.rate();
  double y = com_point[1]*robot.rate();
  double x = com_point[0]*robot.rate();
  double th = 0;

  std::cout << "init" << com_point << std::endl;

  ros::ServiceServer trajectory_service =
      n.advertiseService<custom_services::updateGains::Request,
                         custom_services::updateGains::Response>(
          "trajectory", boost::bind(&wheelHandler, _1, _2, &x, &y, &z, &th));

  // Set initaial HC tasks
  RigidBodyDynamics::Math::VectorNd gain(1);
  gain << 1;
  hierarchical_controller.addTask(&constraints_task, gain, 0, 1e-6);
  gain << 100;
  hierarchical_controller.addTask(&pelvis_hight, gain, 1, 1e-6);
  gain << 100;
  hierarchical_controller.addTask(&pelvis_orientation, gain, 2, 1e-6);

  //#ifdef VISUALIZATION_TOOLS
  // create trackers
//  mwoibn::visualization_tools::RvizTrackPoint tracker("rviz/com_tracker");
//  tracker.initMarker(mwoibn::visualization_tools::Utils::TYPE::POINT, "world",
//                     0.002, 0.002,
//                     0.002); // nr.0 - Pelvis reference trajectory
//  tracker.initMarker(mwoibn::visualization_tools::Utils::TYPE::POINT, "world",
//                     0.004, 0.004,
//                     0.004); // nr.1 - actual pelvis position


  //#endif
  //#ifdef VISUALIZATION_TOOLS
//  robot.command.set(robot.state.get(mwoibn::robot_class::INTERFACE::POSITION),
//                    mwoibn::robot_class::INTERFACE::POSITION);
  //    std::cout << "com_point\n" << com_point << std::endl;

  double eps = 0.005;
  std::cout << "orientation reference\n " << pelvis_orientation.getReference(0) << std::endl;
  std::cout << "orientation offset\n " << pelvis_orientation.getOffset(0) << std::endl;

   while (ros::ok())
  {
/*
    if (std::fabs(hight - com_point[2]) > eps){
      if (hight - com_point[2] > 0)
        com_point[2] += eps;
      else
        com_point[2] -= eps;
    }
    else
      com_point[2] = hight;

    if (std::fabs(y - com_point[1]) > eps){
      if (y - com_point[1] > 0)
        com_point[1] += eps;
      else
        com_point[1] -= eps;
    }
    else
      com_point[1] = y;

    if (std::fabs(x - com_point[0]) > eps){
      if (x - com_point[0] > 0)
        com_point[0] += eps;
      else
        com_point[0] -= eps;
    }
    else
      com_point[0] = x;
    */
    com_point[0] += x*robot.rate();
    com_point[1] += y*robot.rate();
    com_point[2] += z*robot.rate();
    
//    std::cout << "com_point" << std::endl;

//    std::cout << com_point << std::endl;
    pelvis_hight.setReference(com_point);
    pelvis_orientation.setReference(
      0, pelvis_orientation.getOffset(0)*mwoibn::Quaternion::fromAxisAngle(axis, th));
    command =
        hierarchical_controller.update();

    robot.command.set(command, mwoibn::robot_class::INTERFACE::VELOCITY);
//     std::cout << "velocity" << command.head(6) << std::endl;

    command = command * robot.rate() +
        robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);

    robot.command.set(command, mwoibn::robot_class::INTERFACE::POSITION);

    robot.update();
  }
}

bool wheelHandler(custom_services::updateGains::Request& req,
                  custom_services::updateGains::Response& res,
                  double* x, double* y, double* z, double* th)
{

  *x =  req.a_p/100.0;
  *y = req.a_d/100.0;
  *z = req.j_p/100.0;
  *th = req.j_d/100.0;

  res.success = true;
  return true;
}
