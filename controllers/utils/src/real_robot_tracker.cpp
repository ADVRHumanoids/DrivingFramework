#include <mwoibn/robot_class/robot_ros_nrt.h>

//#ifdef VISUALIZATION_TOOLS
#include <mwoibn/visualization_tools/rviz_track_point.h>
#include <mwoibn/point_handling/robot_points_handler.h> //??
//#endif



int main(int argc, char** argv)
{

  ros::init(argc, argv, "center_of_mass_publisher"); // initalize node
  ros::NodeHandle n;

  // load robot configuration and set it up to the starting position
  mwoibn::robot_class::RobotRosNRT robot("/home/user/malgorzata/workspace/src/locomotion_framework/configs/mwoibn_v2.yaml" ,"higher_scheme");// THIS IS NOT A CORRECT SET_UP

  mwoibn::visualization_tools::RvizTrackPoint tracker("rviz/com_tracker_2");
  tracker.initMarker(mwoibn::visualization_tools::Utils::TYPE::POINT, "world", 0.002, 0.002,
                     0.002); // nr.0 - real(simulated) robot CoM reference trajectory
  tracker.setColor(0, 1, 0, 0);

  Eigen::Vector3d com_point;

  for (int i = 0; i < robot.contacts().size(); i++)
  {
    robot.contacts().contact(i).activate();
  }

  while (robot.isRunning())
  {

    robot.centerOfMass().update(false);

    com_point.head(2) = robot.centerOfMass().get().head(2);
    com_point[2] = 0;

    tracker.updateMarker(0, com_point[0], com_point[1], com_point[2]);
    tracker.publish();

    robot.update();
  }

}

