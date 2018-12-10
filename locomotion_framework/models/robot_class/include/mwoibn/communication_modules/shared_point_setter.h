#ifndef __MWOIBN__COMMUNICATION_MODULES__SHARED_POINT_SETTER_H
#define __MWOIBN__COMMUNICATION_MODULES__SHARED_POINT_SETTER_H

#include "mwoibn/communication_modules/basic_point.h"
#include "mwoibn/communication_modules/shared.h"
#include "mwoibn/point_handling/state.h"

namespace mwoibn
{
namespace communication_modules
{

class SharedPointSetter : public BasicPoint
{

public:
  // for now only full robot is supported for this controller
  SharedPointSetter(YAML::Node config, mwoibn::communication_modules::Shared& shared, mwoibn::point_handling::State& point)
      : BasicPoint(point, config), _shared(shared)
  {
    if(!config["name"]) throw(std::invalid_argument("Missing required parameter: name"));

    if(!config["interface"]) throw(std::invalid_argument("Missing required parameter: interface"));

    if(config["interface"].as<std::string>() == "world") _world = true;
    else if(config["interface"].as<std::string>() == "fixed") _world = false;
    else throw(std::invalid_argument("Shared point setter: Unknow 'interface': " + config["interface"].as<std::string>()));

    _init(config["name"].as<std::string>());
  }

  SharedPointSetter(SharedPointSetter&& other)
      : BasicPoint(other), _shared(other._shared), _world(other._world), _filtered(other._filtered), _name(other._name)
  {
  }
  SharedPointSetter(SharedPointSetter& other)
      : BasicPoint(other), _shared(other._shared), _world(other._world), _filtered(other._filtered), _name(other._name)
  {
  }

  virtual ~SharedPointSetter() {}

  virtual bool run();
  virtual bool initialize(){ _initialized = true;}

protected:
  mwoibn::VectorN _filtered;
  mwoibn::communication_modules::Shared& _shared;
  std::string _name;
  bool _world;

  virtual void _init(std::string name);
};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
