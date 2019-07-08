#ifndef COMMUNICATION_MODULES_CUSTOM_CONTROLLER_H
#define COMMUNICATION_MODULES_CUSTOM_CONTROLLER_H

#include "mwoibn/common/all.h"
#include "mwoibn/communication_modules/basic_controller.h"
#include "ros/ros.h"
#include <custom_messages/CustomCmnd.h>
#include <custom_services/loadGains.h>
#include <custom_services/getJointNames.h>
#include <custom_services/updateGains.h>

#include <std_srvs/SetBool.h>
#include <custom_controller/controller_utils.h>
#include "mwoibn/communication_modules/ros_feedback.h"

namespace mwoibn
{
namespace communication_modules
{

class CustomController : public BasicController
{

public:
  CustomController(mwoibn::robot_class::State& command,
                 mwoibn::robot_class::State& lower_limits,
                 mwoibn::robot_class::State& upper_limits,
                   mwoibn::robot_class::BiMap& map, std::string topic,
                   bool position = true, bool velocity = false,
                   bool torque = true)
      : BasicController(command, map, position, velocity, torque), _lower_limits(lower_limits),
        _upper_limits(upper_limits)
  {
    _init(map.reversed(), map.getDofs(), topic);

    std::cout << "Loaded direct controller " << std::endl;
  }

  CustomController(mwoibn::robot_class::State& command,
                 mwoibn::robot_class::State& lower_limits,
                 mwoibn::robot_class::State& upper_limits,
                   mwoibn::robot_class::BiMap&& map, std::string topic,
                   bool position = true, bool velocity = false,
                   bool torque = true)
      : BasicController(command, map, position, velocity, torque), _lower_limits(lower_limits),
        _upper_limits(upper_limits)
  {
    _init(map.reversed(), map.getDofs(), topic);

    std::cout << "Loaded direct controller " << std::endl;
  }

  CustomController(CustomController& other)
      : BasicController(other), _node(other._node), _des_q(other._des_q), _name(other._name), _lower_limits(other._lower_limits),
        _upper_limits(other._upper_limits)
  {
    std::string topic = other._command_pub.getTopic();
    other._command_pub.shutdown();

    _command_pub =
        _node.advertise<custom_messages::CustomCmnd>(topic, 1);

    std::string service = other._load_gains_srv.getService();
    other._load_gains_srv.shutdown();
    _load_gains_srv =
                    _node.advertiseService<custom_services::loadGains::Request,
                                       custom_services::loadGains::Response>(
                            service , boost::bind(&CustomController::loadGains, this, _1, _2));


    }

  CustomController(CustomController&& other)
      : BasicController(other), _node(other._node), _des_q(other._des_q), _name(other._name), _lower_limits(other._lower_limits),
        _upper_limits(other._upper_limits)
  {
    std::string topic = other._command_pub.getTopic();
    other._command_pub.shutdown();

    _command_pub =
        _node.advertise<custom_messages::CustomCmnd>(topic, 1);

    std::string service = other._load_gains_srv.getService();
    other._load_gains_srv.shutdown();
    _load_gains_srv =
                    _node.advertiseService<custom_services::loadGains::Request,
                                       custom_services::loadGains::Response>(
                            service , boost::bind(&CustomController::loadGains, this, _1, _2));


   }

  CustomController(
      mwoibn::robot_class::State& command,
      mwoibn::robot_class::State& lower_limits,
      mwoibn::robot_class::State& upper_limits, mwoibn::robot_class::BiMap& map,
      YAML::Node config)
      : BasicController(command, map, config), _lower_limits(lower_limits),
        _upper_limits(upper_limits)
  {
    _init(config);

  }

  CustomController(
      mwoibn::robot_class::State& command,
      mwoibn::robot_class::State& lower_limits,
      mwoibn::robot_class::State& upper_limits, mwoibn::robot_class::BiMap&& map,
      YAML::Node config)
      : BasicController(command, map, config), _lower_limits(lower_limits),
        _upper_limits(upper_limits)
  { _init(config);}

  virtual ~CustomController() {}

  virtual bool run();

  bool loadGains(custom_services::loadGains::Request& req, custom_services::loadGains::Response& res);

  //  ros::ServiceClient set_feed_forward;

protected:
  //  bool _feed_forward;
  ros::NodeHandle _node;

  ros::Publisher _command_pub;
  custom_messages::CustomCmnd _des_q;
  ros::ServiceServer _load_gains_srv;
  std::string _name;
  const mwoibn::robot_class::State& _lower_limits;
  const mwoibn::robot_class::State& _upper_limits;

  template <typename Vector>
  void _init(const Vector& urdf_rbdl, unsigned int rbdl_dofs,
             const std::string& name,
             std::string command = "/command",
             std::string service = "/set_ff_torque")
  {

    _command_pub =
        _node.advertise<custom_messages::CustomCmnd>(name + command, 1);

    ros::ServiceClient set_feed_forward =
        _node.serviceClient<std_srvs::SetBool>(name + service);


    std_srvs::SetBool srv;
    srv.request.data = true;

    if (!set_feed_forward.call(srv))
    {
      std::cout << "WARNING: Couldn't initialize a feed forward term from "
                   "service:\n\t " << name << service << std::endl;
    }

    _load_gains_srv =
            _node.advertiseService<custom_services::loadGains::Request,
                               custom_services::loadGains::Response>(
                    name+"/load_gains",
                    boost::bind(&CustomController::loadGains, this, _1, _2));

    _map = _initMap(urdf_rbdl, rbdl_dofs);
  }

  template <typename Vector>

  mwoibn::robot_class::BiMap _initMap(const Vector& external_map, int dofs)
  {
      custom_controller::MapToUrdf map;

      Eigen::VectorXi urdf = mwoibn::robot_class::NON_EXISTING *
                             Eigen::VectorXi::Ones(external_map.size());

      _dofs = map.getControllerDofs();

      _des_q.position.resize(_dofs, 0);
      _des_q.effort.resize(_dofs, 0);

      Eigen::VectorXi controller(_dofs);

      for (int i = 0; i < _dofs; i++)
        controller[i] =
            i; // fill controller vector with the sucessive integer numbers

      map.mapFromController(controller, urdf);

      mwoibn::VectorInt rbdl_controller =
          mwoibn::robot_class::NON_EXISTING * Eigen::VectorXi::Ones(dofs);

      for (int i = 0; i < external_map.size(); i++)
      {
        if (external_map[i] != mwoibn::robot_class::NON_EXISTING)
          rbdl_controller[external_map[i]] = urdf[i];
      }

      return mwoibn::robot_class::BiMap("controller", rbdl_controller);
  }



  void _init(YAML::Node config)
  {
    std::cout << "Loading direct controller to the robot" << std::endl;

    if (!config["name"])
      throw(std::invalid_argument("Required argument \"name\" is missing."));

    _name = config["name"].as<std::string>();
    std::string sink =
        (config["sink"]) ? config["sink"].as<std::string>() : "/command";
    std::string service = (config["ff_service"])
                              ? config["ff_service"].as<std::string>()
                              : "/set_ff_torque";

    _init(_map.reversed(), _map.getDofs(), config["name"].as<std::string>(), sink,
          service);


    std::cout << "Loaded direct controller " << config["name"] << std::endl;
  }

  void _limit(mwoibn::Interface interface)
  {

    for (int i = 0; i < _command[interface].size(); i++)
    {
      if(_map.get()[i] == mwoibn::NON_EXISTING) continue;
      if (_lower_limits[interface].get(i) == mwoibn::NON_EXISTING)
          //std::cout << interface << "\t" << i << std::endl;
          continue;
      if (_command[interface].get(i) < _lower_limits[interface].get(i)){
          _command[interface].set(_lower_limits[interface].get(i), i);
      }
      else if (_command[interface].get(i) > _upper_limits[interface].get(i)){
               _command[interface].set(_upper_limits[interface].get(i), i);
      }
    }
  }


};
}
}

#endif // COMMUNICATION_MODULES_CUSTOM_CONTROLLER_H
