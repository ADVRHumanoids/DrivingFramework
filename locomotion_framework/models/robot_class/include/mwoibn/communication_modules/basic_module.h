#ifndef COMMUNICATION_MODULES_BASIC_MODULE_H
#define COMMUNICATION_MODULES_BASIC_MODULE_H

#include "mwoibn/robot_class/robot_class.h"
#include "mwoibn/robot_class/state.h"
#include "mwoibn/robot_class/map.h"

#include <rbdl/rbdl.h>

namespace mwoibn {

namespace communication_modules {

class BasicModule{

public:
  BasicModule(mwoibn::robot_class::State& command, mwoibn::robot_class::BiMap map, bool position, bool velocty, bool torque, bool direct): _command(command), _map(map), _position(position), _velocity(velocty), _torque(torque){

   _dofs = (direct)?_map.getDofs():_map.getDofsReversed();

  }

  BasicModule(mwoibn::robot_class::State& command, mwoibn::robot_class::BiMap map, bool direct, YAML::Node config): _command(command), _map(map){

  if(!config["interface"]) throw(std::invalid_argument("Missing required parameter: interface"));
  if(!config["interface"]["position"]) throw(std::invalid_argument("Missing required parameter: interface:position"));
  if(!config["interface"]["velocity"]) throw(std::invalid_argument("Missing required parameter: interface:velocity"));
  if(!config["interface"]["effort"]) throw(std::invalid_argument("Missing required parameter: interface:effort"));
  _position = config["interface"]["position"].as<bool>();
  _velocity = config["interface"]["velocity"].as<bool>();
  _torque   = config["interface"]["effort"].as<bool>();

  _dofs = (direct)?_map.getDofs():_map.getDofsReversed();

  }

  virtual ~BasicModule(){}


  //! Provides a mapping of from RBDL to controller input
  template <typename Vector1, typename Vector2>
  void mapTo(const Vector1& q_rbdl, Vector2& q_controller) const
  {
    _map.mapReversed(q_rbdl, q_controller);
  }
  template <typename Vector1, typename Vector2>
  void mapTo(const Vector1& q_rbdl, Vector2& q_controller, const Vector3& selector) const
  {
    _map.mapReversed(q_rbdl, q_controller, selector);
  }
  //! Provides a mapping of from controller input to RBDL
  template <typename Vector1, typename Vector2>
  void mapFrom(const Vector1& q_controller, Vector2& q_rbdl) const
  {
      _map.mapDirect(q_controller, q_rbdl);
  }

  template <typename Vector1, typename Vector2, typename Vector3>
  void mapFrom(const Vector1& q_controller, Vector2& q_rbdl, const Vector3& selector) const
  {
      _map.mapDirect(q_controller, q_rbdl, selector);
  }

  virtual mwoibn::VectorInt getSelector() const = 0;

  virtual bool update() = 0;

  int getDofs() const {return _dofs;}

protected:
  mwoibn::robot_class::BiMap _map;
  mwoibn::robot_class::State& _command;

  bool _position;
  bool _velocity;
  bool _torque;

  int _dofs;
};


}

}

#endif // COMMUNICATION_MODULE_H
