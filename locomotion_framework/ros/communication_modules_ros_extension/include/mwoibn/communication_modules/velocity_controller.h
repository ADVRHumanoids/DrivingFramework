#ifndef COMMUNICATION_MODULES_VELOCITY_CONTROLLER_H
#define COMMUNICATION_MODULES_VELOCITY_CONTROLLER_H

#include "mwoibn/robot_class/state.h"
#include "mwoibn/communication_modules/basic_controller.h"
#include "ros/ros.h"
#include <custom_messages/CustomCmnd.h>
#include <std_srvs/SetBool.h>
#include <custom_controller/controller_utils.h>

namespace mwoibn
{
namespace communication_modules
{

class VelocityController : public BasicController
{

public:
  VelocityController(mwoibn::robot_class::State& command, mwoibn::robot_class::BiMap map, std::string topic, bool position = false, bool velocity = true, bool torque = false)
      : BasicController(command, map, position, velocity, torque)
  {
    _init(map.reversed(), map.getDofs(), topic);
  }

  VelocityController(mwoibn::robot_class::State& command, mwoibn::robot_class::BiMap map, YAML::Node config): BasicController(command, map, config)
  {
    std::cout << "Loading direct controller to the robot" << std::endl;

    if(!config["name"])
      throw(std::invalid_argument("Required argument \"name\" is missing."));

    std::string sink = (config["sink"]) ? config["sink"].as<std::string>() : "/command";

    _init(map.reversed(), map.getDofs(), config["name"].as<std::string>(), sink);
    _des_q.velocity.resize(_dofs, 0);
    std::cout << "Success" << std::endl;
  }

  virtual ~VelocityController() {}

  virtual bool send();

  //  ros::ServiceClient set_feed_forward;

protected:
  //  bool _feed_forward;
  ros::NodeHandle _node;

  ros::Publisher _command_pub;
  custom_messages::CustomCmnd _des_q;

  template <typename Vector> void _init(const Vector& urdf_rbdl, unsigned int rbdl_dofs, std::string name, std::string command = "/command")
  {
    _command_pub =
        _node.advertise<custom_messages::CustomCmnd>(name + command, 1);

    custom_controller::MapToUrdf map("joint_states", name);

    Eigen::VectorXi urdf = mwoibn::robot_class::NON_EXISTING *
                           Eigen::VectorXi::Ones(urdf_rbdl.size());

    _dofs = map.getControllerDofs();

    Eigen::VectorXi controller(_dofs);

    for (int i = 0; i < _dofs; i++)
      controller[i] =
          i; // fill controller vector with the sucessive integer numbers

    map.mapFromController(controller, urdf);

    mwoibn::VectorInt rbdl_controller =
        mwoibn::robot_class::NON_EXISTING * Eigen::VectorXi::Ones(rbdl_dofs);

    for (int i = 0; i < urdf_rbdl.size(); i++)
    {
      if (urdf_rbdl[i] != mwoibn::robot_class::NON_EXISTING)
        rbdl_controller[urdf_rbdl[i]] = urdf[i];
    }

    std::cout << rbdl_controller << std::endl;

    _map = mwoibn::robot_class::BiMap(name, rbdl_controller);
  }
};
}
}

#endif // COMMUNICATION_MODULES_CUSTOM_CONTROLLER_H