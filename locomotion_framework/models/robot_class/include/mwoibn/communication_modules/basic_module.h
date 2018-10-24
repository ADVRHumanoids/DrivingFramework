#ifndef COMMUNICATION_MODULES_BASIC_MODULE_H
#define COMMUNICATION_MODULES_BASIC_MODULE_H

#include "mwoibn/common/all.h"
#include "mwoibn/common/all.h"
#include "mwoibn/robot_class/map.h"
#include "mwoibn/filters/iir_second_order.h"

#include <rbdl/rbdl.h>

namespace mwoibn
{

namespace communication_modules
{

class BasicModule
{

public:
  BasicModule(mwoibn::robot_class::State& command,
              mwoibn::robot_class::BiMap map, bool position, bool velocty,
              bool torque, bool direct)
      : _command(command), _map(map), _position(position), _velocity(velocty),
        _torque(torque)
  {

    _resize(direct);
  }

  BasicModule(mwoibn::robot_class::State& command,
              mwoibn::robot_class::BiMap map, bool direct, YAML::Node config)
      : _command(command), _map(map)
  {

    if (!config["interface"])
      throw(std::invalid_argument("Missing required parameter: interface"));
    if (!config["interface"]["position"])
      throw(std::invalid_argument(
          "Missing required parameter: interface:position"));
    if (!config["interface"]["velocity"])
      throw(std::invalid_argument(
          "Missing required parameter: interface:velocity"));
    if (!config["interface"]["effort"])
      throw(std::invalid_argument(
          "Missing required parameter: interface:effort"));

    _position = config["interface"]["position"].as<bool>();
    _velocity = config["interface"]["velocity"].as<bool>();
    _torque = config["interface"]["effort"].as<bool>();

    _resize(direct);

    if(config["filter"]){
      if(!config["filter"]["run"])
        throw(std::invalid_argument("Missing required filter parameter: run"));

      _filter = config["filter"]["run"].as<bool>();

      if(_filter){
        if(!config["filter"]["frequency"])
          throw(std::invalid_argument("Missing required filter parameter: frequency"));
        if(!config["filter"]["damping"])
          throw(std::invalid_argument("Missing required filter parameter: damping"));

        if(_position)
          _position_filter_ptr.reset(new mwoibn::filters::IirSecondOrder(_dofs, config["filter"]["frequency"].as<double>(), config["filter"]["damping"].as<double>()));
        if(_velocity)
          _velocity_filter_ptr.reset(new mwoibn::filters::IirSecondOrder(_dofs, config["filter"]["frequency"].as<double>(), config["filter"]["damping"].as<double>()));
        if(_torque)
          _torque_filter_ptr.reset(new mwoibn::filters::IirSecondOrder(_dofs, config["filter"]["frequency"].as<double>(), config["filter"]["damping"].as<double>()));

        std::cout << "Filter has been enabled. Cut-off frequency " << config["filter"]["frequency"] << ", damping " << config["filter"]["damping"] << "." << std::endl;
      }
    }
    else
      _filter = false;

  }

  virtual ~BasicModule() {}

  //! Provides a mapping of from RBDL to controller input
  template <typename Vector1, typename Vector2>
  void mapTo(const Vector1& q_rbdl, Vector2& q_controller) const
  {
    _map.mapReversed(q_rbdl, q_controller);
  }

  template <typename Vector1, typename Vector2>
  void mapTo(const Vector1& q_rbdl, Vector2& q_controller,
             const Vector3& selector) const
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
  void mapFrom(const Vector1& q_controller, Vector2& q_rbdl,
               const Vector3& selector) const
  {
    _map.mapDirect(q_controller, q_rbdl, selector);
  }

  virtual mwoibn::VectorInt getSelector() const = 0;

  virtual bool update() = 0;

  int getDofs() const { return _dofs; }

  bool is(mwoibn::robot_class::INTERFACE interface){

    if(interface == mwoibn::robot_class::INTERFACE::POSITION)
      return _position;
    if(interface == mwoibn::robot_class::INTERFACE::VELOCITY)
      return _velocity;
    if(interface == mwoibn::robot_class::INTERFACE::TORQUE)
      return _torque;

    return false;
  }

  virtual bool raw(mwoibn::VectorN& _raw, mwoibn::robot_class::INTERFACE interface)
  {
      return false;
  }

  std::string getMapName(){ return _map.getName();}

  virtual bool reset(){return true;}
  virtual bool initialized(){return _initialized;}
  virtual bool initialize(){
    _initialized = true;
    return initialized();}

protected:
  mwoibn::robot_class::BiMap _map;
  mwoibn::robot_class::State& _command;
  std::unique_ptr<mwoibn::filters::IirSecondOrder> _position_filter_ptr;
  std::unique_ptr<mwoibn::filters::IirSecondOrder> _velocity_filter_ptr;
  std::unique_ptr<mwoibn::filters::IirSecondOrder> _torque_filter_ptr;


  void _resize(bool direct)
  {
    _dofs = (direct) ? _map.getDofs() : _map.getDofsReversed();
  }

  bool _position;
  bool _velocity;
  bool _torque;
  bool _filter;
  bool _initialized = false;
  int _dofs;
};
}
}

#endif // COMMUNICATION_MODULE_H
