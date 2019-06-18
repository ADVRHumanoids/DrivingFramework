#include "mwoibn/communication_modules/xbot_lower_level.h"

bool mwoibn::communication_modules::XBotLowerLevel::run()
{
  if (_position)
  {
    _robot.getPositionReference(pub);
//    std::cout << "position before\t" << pub.transpose() << std::endl;

    _limit("POSITION");
    mapTo(_command.position.get(), pub);
//    std::cout << "position after\t" << pub.transpose() << std::endl;
    
    _robot.setPositionReference(pub);
  }
  if (_velocity)
  {
    _robot.getVelocityReference(pub);
//    std::cout << "velocity before\t" << pub.transpose() << std::endl;

    _limit("VELOCITY");
    mapTo(_command.velocity.get(), pub);

//    std::cout << "velocity after\t" << pub.transpose() << std::endl;
    _robot.setVelocityReference(pub);
  }
  if (_torque)
  {
    _robot.getEffortReference(pub);

//    std::cout << "torque before\t" << pub.transpose() << std::endl;
    _limit("TORQUE");
    mapTo(_command.torque.get(), pub);
//    std::cout << "torque after\t" << pub.transpose() << std::endl;
    _robot.setEffortReference(pub);
  }

  _robot.setStiffness(stiffness);
  _robot.setDamping(damping);
  return true;
}
