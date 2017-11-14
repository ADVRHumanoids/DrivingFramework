#include "mwoibn/communication_modules/xbot_lower_level.h"

bool mwoibn::communication_modules::XBotLowerLevel::send()
{
  if (_position)
  {
    _robot.getPositionReference(pub);

    _limit(mwoibn::robot_class::INTERFACE::POSITION);
    mapTo(_command.get(mwoibn::robot_class::INTERFACE::POSITION), pub);
    _robot.setPositionReference(pub);
  }
  if (_velocity)
  {
    _robot.getVelocityReference(pub);
    _limit(mwoibn::robot_class::INTERFACE::VELOCITY);
    mapTo(_command.get(mwoibn::robot_class::INTERFACE::VELOCITY), pub);
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
