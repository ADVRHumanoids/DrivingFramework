#include "mwoibn/communication_modules/xbot_lower_level.h"

bool mwoibn::communication_modules::XBotLowerLevel::send()
{
//  std::cout << "send()\n";
    if (_position)
    {
     // pub.setZero();
       _robot.getPositionReference(pub);

      mapTo(_command.get(mwoibn::robot_class::INTERFACE::POSITION),
                      pub);
      _robot.setPositionReference(pub);
    }
    if (_velocity)
    {
//      std::cout << "velocity\n";

      //pub.setZero();
      _robot.getVelocityReference(pub);
      mapTo(_command.get(mwoibn::robot_class::INTERFACE::VELOCITY),
                      pub);
      _robot.setVelocityReference(pub);
    }
    if (_torque)
    {
      //pub.setZero(_dofs); // maybe remove?
      _robot.getEffortReference(pub);
      mapTo(_command.get(mwoibn::robot_class::INTERFACE::TORQUE),
                      pub);
      _robot.setEffortReference(pub);
    }

    return true;
}
