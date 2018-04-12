#include "mwoibn/communication_modules/xbot_lower_level.h"

bool mwoibn::communication_modules::XBotLowerLevel::send()
{
  if (_position)
  {
    _robot.getPositionReference(pub);
//    std::cout << "position before\t" << pub.transpose() << std::endl;

    _limit(mwoibn::robot_class::INTERFACE::POSITION);
    mapTo(_command.get(mwoibn::robot_class::INTERFACE::POSITION), pub);

//    std::cout << "position after\t" << pub.transpose() << std::endl;
    _robot.setPositionReference(pub);
  }

  if (_velocity)
  {
    _robot.getVelocityReference(pub);
//    std::cout << "velocity before\t" << pub.transpose() << std::endl;
//    std::cout << "velocity command\t" << _command.get(mwoibn::robot_class::INTERFACE::VELOCITY).transpose() << std::endl;

    _limit(mwoibn::robot_class::INTERFACE::VELOCITY);
    mapTo(_command.get(mwoibn::robot_class::INTERFACE::VELOCITY), pub);

//    std::cout << "velocity after\t" << pub.transpose() << std::endl;
    _robot.setVelocityReference(pub);
  }
  if (_torque)
  {
    _robot.getEffortReference(pub);

    _limit(mwoibn::robot_class::INTERFACE::TORQUE);
    mapTo(_command.get(mwoibn::robot_class::INTERFACE::TORQUE), pub);

    _robot.setEffortReference(pub);
  }

  _robot.setStiffness(stiffness);
  _robot.setDamping(damping);

  return true;
}
