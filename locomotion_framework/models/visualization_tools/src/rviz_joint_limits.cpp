#include "mwoibn/visualization_tools/rviz_joint_limits.h"

bool mwoibn::visualization_tools::RvizJointLimits::updateModel()
{

  // green 50, 205, 50, 1
  // yellow 255, 255, 102
  // orange 255,140,0
  // red  246, 34, 34

  mwoibn::VectorN robot_state = _robot.state.position.get();
  int i = 0;
  for (auto& limit : _limits)
  {
    for (auto& alert : _alert_level)
    {


      if ((robot_state[std::get<0>(limit)] <
           std::get<0>(alert) * std::get<2>(limit)) &
          (robot_state[std::get<0>(limit)] >
           std::get<0>(alert) * std::get<1>(limit)))
      {
        _msg.markers[i].color.r = std::get<1>(alert);
        _msg.markers[i].color.g = std::get<2>(alert);
        _msg.markers[i].color.b = std::get<3>(alert);
//        std::cout << "found" << "\n";
//        std::cout <<  _msg.markers[std::get<3>(limit)].header.frame_id  << "\n";
//        std::cout <<  std::get<0>(alert)  << "\n";
//        std::cout <<  std::get<1>(limit)  << "\n";
//        std::cout <<  std::get<2>(limit)  << "\n";
//        std::cout <<  robot_state[std::get<0>(limit)]  << "\n";
        i++;

        break;
      }
    }
  }

  return true;
}
