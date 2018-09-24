#ifndef COMMUNICATION_MODULES_CUSTOM_CONTROLLER_H
#define COMMUNICATION_MODULES_CUSTOM_CONTROLLER_H

#include "mwoibn/robot_class/state.h"
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
                   mwoibn::robot_class::BiMap map, std::string topic,
                   bool position = true, bool velocity = false,
                   bool torque = true)
      : BasicController(command, map, position, velocity, torque)
  {
    _init(map.reversed(), map.getDofs(), topic);
  }

  CustomController(
      mwoibn::robot_class::State& command, mwoibn::robot_class::BiMap map,
      YAML::Node config,
      mwoibn::communication_modules::BasicFeedback* feedback = nullptr,
      const mwoibn::robot_class::BiMap* feedback_map = nullptr)
      : BasicController(command, map, config)
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

    _init(map.reversed(), map.getDofs(), config["name"].as<std::string>(), sink,
          service);

    _des_q.position.resize(_dofs, 0);
    _des_q.effort.resize(_dofs, 0);

    mwoibn::VectorN q_0;

    if (feedback != nullptr &&
        feedback->raw(q_0, mwoibn::robot_class::INTERFACE::POSITION))
    {
      if (feedback_map == nullptr)
        _des_q.position =
            std::vector<double>(q_0.data(), q_0.data() + q_0.size());
      else
      {
        mwoibn::robot_class::BiMap temp_map = _initMap(feedback_map->get(), feedback_map->getDofs());

        temp_map.mapReversed(q_0, _des_q.position);
      }
    }
    else
      throw(std::runtime_error("Couldn't initialize robot state"));



    std::cout << "Loaded direct controller " << config["name"] << std::endl;
  }

  virtual ~CustomController() {}

  virtual bool send();

  bool loadGains(custom_services::loadGains::Request& req, custom_services::loadGains::Response& res);

  //  ros::ServiceClient set_feed_forward;

protected:
  //  bool _feed_forward;
  ros::NodeHandle _node;

  ros::Publisher _command_pub;
  custom_messages::CustomCmnd _des_q;
  ros::ServiceServer _load_gains_srv;
  std::string _name;

  template <typename Vector>
  void _init(const Vector& urdf_rbdl, unsigned int rbdl_dofs,
             const std::string& name, std::string command = "/command",
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
};
}
}

#endif // COMMUNICATION_MODULES_CUSTOM_CONTROLLER_H
