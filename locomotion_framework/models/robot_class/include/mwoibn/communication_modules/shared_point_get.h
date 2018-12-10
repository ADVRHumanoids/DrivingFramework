#ifndef __MOWIBN__COMMUNICATION_MODULES__SHARED_POINT_GET_H
#define __MOWIBN__COMMUNICATION_MODULES__SHARED_POINT_GET_H

#include "mwoibn/communication_modules/basic_point.h"
#include "mwoibn/communication_modules/shared.h"
#include "mwoibn/point_handling/point.h"


namespace mwoibn
{
namespace communication_modules
{
class SharedPointGet : public BasicPoint
{

public:
  // for now only full robot is supported for this controller
  SharedPointGet(YAML::Node config, mwoibn::communication_modules::Shared& shared, mwoibn::point_handling::State& point)
      : BasicPoint(point, config), _shared(shared){
        _init(config);
      }


  SharedPointGet(SharedPointGet& other)
      : BasicPoint(other), _shared(other._shared), _name(other._name)
  {  }

  SharedPointGet(SharedPointGet&& other)
      : BasicPoint(other), _shared(other._shared), _name(other._name)
  {  }

  virtual ~SharedPointGet() {}
  virtual bool initialized() { return _initialized; }
  virtual bool run() {
    _initialized = true;

    if (_world)
      _point.setWorld(_shared[_name+".world"]);
    else
      _point.setFixed(_shared[_name+".fixed"]);

      return true;
  }

protected:
  mwoibn::communication_modules::Shared& _shared;
  bool _initialized = false;
  std::string _name;
  bool _world;

  void _init(YAML::Node config)
  {
    if (!config["name"])
      throw(std::invalid_argument("Missing required parameter: name"));

      _name = config["name"].as<std::string>();

      if(!config["interface"]) throw(std::invalid_argument("Missing required parameter: interface"));

      if(config["interface"].as<std::string>() == "world") _world = true;
      else if(config["interface"].as<std::string>() == "fixed") _world = false;
      else throw(std::invalid_argument("Shared point get: Unknow 'interface': " + config["interface"].as<std::string>()));


      if(!_shared.startsWith(_name))
        throw(std::invalid_argument("Shared object " + _name + std::string(" does not exists.")));
      if(_world && !_shared.has(_name+".world"))
        throw(std::invalid_argument("World interface for shared object " + _name + std::string(" has not been initialized.")));
      if(!_world && !_shared.has(_name+".fixed"))
        throw(std::invalid_argument("Fixed interface for shared object " + _name + std::string(" has not been initialized.")));


    std::cout << "Loaded Shared point get " << config["name"] << std::endl;
  }

};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
