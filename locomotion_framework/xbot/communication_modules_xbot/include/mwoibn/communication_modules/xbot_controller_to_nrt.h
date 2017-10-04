#ifndef COMMUNICATION_MODULES_XBOT_CONTROLLER_TO_NRT_H
#define COMMUNICATION_MODULES_XBOT_CONTROLLER_TO_NRT_H

#include "mwoibn/communication_modules/basic_controller.h"
#include <XBotCore-interfaces/XDomainCommunication.h>


namespace mwoibn
{
namespace communication_modules
{

class XBotControllerToNRT : public BasicController
{
public:
  XBotControllerToNRT(mwoibn::robot_class::State& command,
                 mwoibn::robot_class::BiMap map, YAML::Node config)
      : BasicController(command, map, config)
  {

    std::cout << "Loading controller to non-real-time - " << config["name"].as<std::string>() << "\n";

    if (!config["sink"])
      throw(std::invalid_argument("Please define sink for the real-time controller " + config["name"].as<std::string>() + ".\n"));

   if(_position){
     _pub_position.init(config["sink"].as<std::string>()+"_position");
     std::cout << "\tInitialized position interface in " << config["sink"].as<std::string>() << "_position\n";
   }
     if(_velocity){
     _pub_velocity.init(config["sink"].as<std::string>()+"_velocity");
     std::cout << "\tInitialized velocity interface in " << config["sink"].as<std::string>() << "_velocity\n";
     }
     if(_torque){
     _pub_torque.init(config["sink"].as<std::string>()+"_torque");
     std::cout << "\tInitialized torque interface in " << config["sink"].as<std::string>() << "_torque\n";
     }

     _data.setZero();
     _data[check] = mwoibn::IS_VALID;
     std::cout << "\tSuccess" << std::endl;
  }

  virtual ~XBotControllerToNRT(){}

  virtual bool send(){

    if (_position){
      mapTo(_command.get(mwoibn::robot_class::INTERFACE::POSITION),
                      _data);
      _pub_position.write(_data);
    }
    if (_velocity){
      mapTo(_command.get(mwoibn::robot_class::INTERFACE::VELOCITY),
                      _data);
      _pub_velocity.write(_data);

    }
    if (_torque){
      mapTo(_command.get(mwoibn::robot_class::INTERFACE::TORQUE),
                      _data);
      _pub_torque.write(_data);
    }

      return true;
  }

protected:
    mwoibn::VectorRT _data;

    XBot::PublisherRT<mwoibn::VectorRT>_pub_position;
    XBot::PublisherRT<mwoibn::VectorRT> _pub_velocity;
    XBot::PublisherRT<mwoibn::VectorRT> _pub_torque;
    const int check = RT_SIZE - 1;

};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
