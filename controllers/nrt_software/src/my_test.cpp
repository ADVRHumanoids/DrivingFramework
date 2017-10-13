#include <mwoibn/robot_class/robot_ros_nrt.h>
#include <mwoibn/robot_class/robot_xbot_nrt.h>
#include <mwoibn/robot_class/robot_xbot_rt.h>

#include <mwoibn/hierarchical_control/hierarchical_controller.h>
#include <mwoibn/hierarchical_control/joint_positions_task.h>
#include <mwoibn/hierarchical_control/center_of_mass_task.h>
#include <mwoibn/hierarchical_control/constraints_task.h>
#include <mwoibn/hierarchical_control/controller_task.h>
#include <mwoibn/hierarchical_control/cartesian_selective_task.h>
#include <mwoibn/hierarchical_control/orientation_world_task.h>
#include <mwoibn/hierarchical_control/orientation_selective_task.h>

//#ifdef VISUALIZATION_TOOLS
//#include <mwoibn/visualization_tools/rviz_track_point.h>

#include <mwoibn/point_handling/robot_points_handler.h>
//#endif
#include <custom_services/updatePDGains.h>
#include <custom_services/updatePrint.h>

// trajectory generation
#include <mwoibn/reference_generation/line.h>
#include <mwoibn/reference_generation/local_circle.h>
#include <mgnss/communication/basic_handler.h>

// temp
#include <MathGeoLib/Algorithm/Random/LCG.h>
#include <MathGeoLib/Geometry/GeometryAll.h>

#include <mwoibn/eigen_utils/eigen_utils.h>

int main(int argc, char** argv)
{

  ros::init(argc, argv,
            "gravity_test"); // initalize node needed for the service

  ros::NodeHandle n;
  ros::Rate rate(200);

  std::string path = "/home/malgorzata/catkin_ws/src/";
//  mwoibn::robot_class::RobotRosNRT robot(path+"DrivingFramework/locomotion_framework/configs/mwoibn_v2.yaml", "higher_scheme", path+"controllers/nrt_software/configs/centralized_controller.yaml");
   mwoibn::robot_class::RobotXBotNRT robot(path+"DrivingFramework/locomotion_framework/configs/mwoibn_v2.yaml", "default", path+"DrivingFramework/controllers/nrt_software/configs/centralized_controller.yaml");
  //  robot.contacts().contact(1).deactivate();

    std::cout << "Dofs: " << robot.getDofs() << std::endl;
  mwoibn::VectorN command(robot.getDofs()), velocities(robot.getDofs()) ;

//  command << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//      3.14 / 2, 3.14 / 4, 3.14 / 4, 0, 0, 0,
//      -3.14 / 2, -3.14 / 4, -3.14 / 4, 0, 0, 0,
//      -3.14 / 2, -3.14 / 4, -3.14 / 4, 0, 0, 0,
//      3.14 / 2, 3.14 / 4, 3.14 / 4, 0, 0, 0,
//      0, 0.0, -0.5236, -0.5236, -0.7854, 0.0, -0.7854, 0.0,
//      0.0, 0.0, 0.5236, 0.5236, 0.7854,  0.0, 0.7854,  0.0,
//      0.0;

//  command << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.14 / 2, 3.14 / 4, 3.14 / 4, 0, 0,
//      0, -3.14 / 2, -3.14 / 4, -3.14 / 4, 0, 0, 0, -3.14 / 2, -3.14 / 4,
//      -3.14 / 4, 0, 0, 0, 3.14 / 2, 3.14 / 4, 3.14 / 4, 0, 0, 0, 0, 0.0,
//      -0.5236, -0.5236, -0.7854, 0.0, -0.7854, 0.0, 0.0, 0.5236, 0.5236, 0.7854,
//      0.0, 0.7854, 0.0;


//  command << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//      0, -1.57, -2.41, -0.90, 0, 0,
//      0,  1.57,  2.41,  0.90, 0, 0,
//      0,  1.57,  2.41,  0.90, 0, 0,
//      0, -1.57, -2.41, -0.90, 0, 0,
//      0.0, 0.0, -0.5236, -0.5236, -0.7854, 0.0, -0.7854,
//      0.0, 0.0,  0.5236,  0.5236,  0.7854, 0.0, 0.7854, 0.0;

//    command << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//        3.14 / 2, 3.14 / 4, 3.14 / 4, 0, 0, 0,
//        -3.14 / 2, -3.14 / 4, -3.14 / 4, 0, 0, 0,
//        -3.14 / 2, -3.14 / 4, -3.14 / 4, 0, 0, 0,
//        3.14 / 2, 3.14 / 4, 3.14 / 4, 0, 0, 0,

  command << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0, -1.57, -1.57, -0.0, 0, -0.5,
      0,  1.57,  1.57,  0.0, 0, 0.5,
      0,  1.57,  1.57,  0.0, 0, 0.5,
      0, -1.57, -1.57, -0.0, 0, -0.5;

//  command << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//      0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  velocities << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0, 0.0, 0.0, -0.0, 0, -0.0,
      0, 0.0, 0.0,  0.0, 0,  0.0,
      0, 0.0, 0.0,  0.0, 0, -0.0,
      0, 0.0, 0.0, -0.0, 0,  0.0;

//    command << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//        0, -1.57, -2.41, -0.90, 0, 0,
//        0,  1.57,  2.41,  0.90, 0, 0,
//        0,  1.57,  2.41,  0.90, 0, 0,
//        0, -1.57, -2.41, -0.90, 0, 0,

//  command << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//            0, -1.57, -2.41, -0.90, 0, 0,
//            0,  1.57,  2.41,  0.90, 0, 0,
//            0,  1.57,  2.41,  0.90, 0, 0,
//            0, -1.57, -2.41, -0.90, 0, 0,
//      0, 0.0, -0.5236, -0.5236, -0.7854, 0.0, -0.7854, 0.0,
//      0.0, 0.0, 0.5236, 0.5236, 0.7854,  0.0, 0.7854,  0.0,
//      0.0;


  robot.command.set(command, mwoibn::robot_class::INTERFACE::POSITION);
  robot.command.set(velocities, mwoibn::robot_class::INTERFACE::VELOCITY);

  while (robot.isRunning())
  {
    robot.update();
  }
}
