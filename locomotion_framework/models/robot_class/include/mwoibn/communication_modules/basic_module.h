#ifndef COMMUNICATION_MODULES_BASIC_MODULE_H
#define COMMUNICATION_MODULES_BASIC_MODULE_H

#include "mwoibn/communication_modules/communication_base.h"
#include "mwoibn/robot_class/map.h"
#include "mwoibn/filters/iir_second_order.h"

#include <rbdl/rbdl.h>

namespace mwoibn
{

namespace communication_modules
{

class BasicModule: public CommunicationBase
{

public:
  BasicModule(mwoibn::robot_class::State& command,
              mwoibn::robot_class::BiMap& map, bool position, bool velocity,
              bool torque, bool direct)
      : CommunicationBase(), _command(command), _map(map), _position(position), _velocity(velocity),
        _torque(torque)
  {
    _resize(direct);
  }

  BasicModule(mwoibn::robot_class::State& command,
              mwoibn::robot_class::BiMap&& map, bool position, bool velocity,
              bool torque, bool direct)
      : CommunicationBase(), _command(command), _map(map), _position(position), _velocity(velocity),
        _torque(torque)
  {
    _resize(direct);
  }



  BasicModule(mwoibn::robot_class::State& command,
                mwoibn::robot_class::BiMap& map, bool direct, YAML::Node config)
      : CommunicationBase(), _command(command), _map(map){
        _resize(direct);

          _init(config);
      }

  BasicModule(mwoibn::robot_class::State& command,
                    mwoibn::robot_class::BiMap&& map, bool direct, YAML::Node config)
      : CommunicationBase(), _command(command), _map(map){
        _resize(direct);

              _init(config);
        }



  BasicModule(BasicModule& other)
      : CommunicationBase(other), _command(other._command), _map(other._map),
        _position(other._position), _velocity(other._velocity), _torque(other._torque), _filter(other._filter)
  {
    if(other._position_filter_ptr != nullptr)
      _position_filter_ptr.reset(new mwoibn::filters::IirSecondOrder(*other._position_filter_ptr));
    if(other._velocity_filter_ptr != nullptr)
      _velocity_filter_ptr.reset(new mwoibn::filters::IirSecondOrder(*other._velocity_filter_ptr));
      if(other._torque_filter_ptr != nullptr)
      _torque_filter_ptr.reset(new mwoibn::filters::IirSecondOrder(*other._torque_filter_ptr));
  }



    BasicModule(BasicModule&& other)
        : CommunicationBase(other), _command(other._command), _map(other._map),
          _position(other._position), _velocity(other._velocity), _torque(other._torque), _filter(other._filter)
    {
      if(other._position_filter_ptr != nullptr)
        _position_filter_ptr.reset(new mwoibn::filters::IirSecondOrder(*other._position_filter_ptr));
      if(other._velocity_filter_ptr != nullptr)
        _velocity_filter_ptr.reset(new mwoibn::filters::IirSecondOrder(*other._velocity_filter_ptr));
        if(other._torque_filter_ptr != nullptr)
        _torque_filter_ptr.reset(new mwoibn::filters::IirSecondOrder(*other._torque_filter_ptr));
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

  // virtual mwoibn::VectorInt getSelector() const = 0;

  bool is(mwoibn::Interface interface){

    if(interface == "POSITION")
      return _position;
    if(interface == "VELOCITY")
      return _velocity;
    if(interface == "TORQUE")
      return _torque;

    return false;
  }

  virtual bool raw(mwoibn::VectorN& _raw, mwoibn::Interface interface)
  {
      return false;
  }

  std::string getMapName(){ return _map.getName();}


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

  void _init(YAML::Node config)
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
};
}
}

#endif // COMMUNICATION_MODULE_H
