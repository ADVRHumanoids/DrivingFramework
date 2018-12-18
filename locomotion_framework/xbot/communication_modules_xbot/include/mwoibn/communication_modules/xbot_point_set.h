#ifndef __MWOIBN__COMMUNICATION_MODULES__XBOT_POINT_SET_H
#define __MWOIBN__COMMUNICATION_MODULES__XBOT_POINT_SET_H

#include "mwoibn/communication_modules/basic_point.h"
#include "XBotCore-interfaces/XBotSharedMemory.h"

namespace mwoibn
{
namespace communication_modules
{

class XBotPointSet : public BasicPoint
{
public:
  XBotPointSet(YAML::Node config, mwoibn::point_handling::State& point, XBot::SharedMemory::Ptr shared_memory)
      : BasicPoint(point, config)
  {

    if(!config["name"]) throw(std::invalid_argument("Missing required parameter: name"));

    if(!config["interface"]) throw(std::invalid_argument("Missing required parameter: interface"));

    if(config["interface"].as<std::string>() == "world") _world = true;
    else if(config["interface"].as<std::string>() == "fixed") _world = false;
    else throw(std::invalid_argument("Shared point setter: Unknow 'interface': " + config["interface"].as<std::string>()));


    _init(config["name"].as<std::string>(), shared_memory);

    std::cout << "\tSuccess" << std::endl;
  }

  virtual ~XBotPointSet()
  {
      _state[check] = mwoibn::INVALID;
      _pub.set(_state);
  }

  virtual bool initialize()
  {
    _state[check] = mwoibn::IS_VALID;

    _initialized = true;
    return _initialized;
  }

  virtual bool run()
  {
    initialize();

    if(_world)
      _state.head<6>() = _point.getWorld();
    else
      _state.head<6>() = _point.getFixed();

    _pub.set(_state);

    return true;
  }

protected:
  bool _world;
  mwoibn::VectorRT _state;
  std::string _name;
  XBot::SharedObject<mwoibn::VectorRT> _pub;

  const int check = RT_SIZE - 1;

  virtual void _init(std::string name, XBot::SharedMemory::Ptr shared_memory){

    if(_world){
      _state.head<6>() = _point.getWorld();
      _name = name+".world";
    }
    else{
      _state.head<6>() = _point.getFixed();
      _name = name+".fixed";
    }

    _pub = shared_memory->getSharedObject<mwoibn::VectorRT>(_name);

      std::cout << "\tInitialized point state feedback " << _name << std::endl;
  }

};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
