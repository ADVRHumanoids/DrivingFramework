#ifndef __MWOIBN__COMMUNICATION_MODULES__XBOT_POINT_GET_H
#define __MWOIBN__COMMUNICATION_MODULES__XBOT_POINT_GET_H

#include "mwoibn/communication_modules/basic_point.h"
#include "XBotCore-interfaces/XBotSharedMemory.h"

namespace mwoibn
{
namespace communication_modules
{

class XBotPointGet : public BasicPoint
{
public:
  XBotPointGet(YAML::Node config, mwoibn::point_handling::State& point,
                       XBot::SharedMemory::Ptr shared_memory)
      : BasicPoint(point, config)
  {
    _init(config, shared_memory);
  }

  virtual ~XBotPointGet()
  {  }

  virtual bool initialize()
  {
    _sub.get(_state);
    if (_state[check] != mwoibn::IS_VALID)
      return false;
    return true;
  }

  virtual bool run()
  {
    if(!initialize()) return false;
    _sub.get(_state);

    if (_world)
      _point.setWorld(_state);
    else
      _point.setFixed(_state);

    return true;
  }

protected:
  bool _world;
  mwoibn::VectorRT _state;
  std::string _name;
  XBot::SharedObject<mwoibn::VectorRT> _sub;

  const int check = RT_SIZE - 1;

  virtual void _init(YAML::Node config, XBot::SharedMemory::Ptr shared_memory){

    if (!config["name"])
      throw(std::invalid_argument("Missing required parameter: name"));

      _name = config["name"].as<std::string>();

      if(!config["interface"]) throw(std::invalid_argument("Missing required parameter: interface"));

      if(config["interface"].as<std::string>() == "world") _world = true;
      else if(config["interface"].as<std::string>() == "fixed") _world = false;
      else throw(std::invalid_argument("Shared point get: Unknow 'interface': " + config["interface"].as<std::string>()));


     _sub = shared_memory->getSharedObject<mwoibn::VectorRT>(_name);
     std::cout << "Loaded shared point get " << _name << std::endl;



  }
};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
