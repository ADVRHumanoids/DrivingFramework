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
//    std::cout << "velocity command\t" << _command.velocity.get.transpose() << std::endl;

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


bool mwoibn::communication_modules::XBotLowerLevel::loadGains(custom_services::loadGains::Request& req, custom_services::loadGains::Response& res){
    YAML::Node config;
    try
    {
            config = YAML::LoadFile(req.file);
    }
    catch (const YAML::BadFile& e)
    {
            res.message = "Couldn\t find the configuration file: " + req.file;
            res.success = false;
            return false;
    }
    catch (...)
    {
      res.message = "Unkown error reading file: " + req.file;
      res.success = false;
      return false;
    }

    if(!config[_name]) {
      res.message = "Couldn't find robot configuration in the config file: " + req.file + "\t" + _name;
      res.success = false;
      return false;
    }

    _readGains(config[_name]);




    res.success = true;
  return true;
}
