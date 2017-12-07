#ifndef COMMUNICATION_MODULES_ROS_OPERATIONAL_EULER_H
#define COMMUNICATION_MODULES_ROS_OPERATIONAL_EULER_H

#include "mwoibn/communication_modules/basic_operational_euler.h"
#include "ros/ros.h"
#include <custom_messages/CustomCmnd.h>

namespace mwoibn
{
namespace communication_modules
{
template <typename Message, typename MessagePtr>
class RosOperationalEuler : public BasicOperationalEuler
{

public:
  // for now only full robot is supported for this controller
  RosOperationalEuler(mwoibn::robot_class::State& command,
                      mwoibn::robot_class::BiMap map, YAML::Node config)
      : BasicOperationalEuler(command, map, config)
  {
    if (!config["source"])
      throw(std::invalid_argument("Missing required parameter: source"));

    _state_sub = _node.subscribe<Message>(
        config["source"].as<std::string>(), 1,
        boost::bind(&RosOperationalEuler::get, this, _1));

    if (!config["reference"])
      throw(std::invalid_argument("Couldn't find an argument [reference]"));

    _ref = config["reference"].as<int>();

    if (!config["convention"])
      throw(std::invalid_argument(
          "Couldn't find a convention specification: [convention]"));
    if (!config["convention"]["position"])
      throw(std::invalid_argument("Couldn't find a position convention "
                                  "specification: [convention][position]"));
    if (!config["convention"]["orientation"])
      throw(std::invalid_argument("Couldn't find an orientation convention "
                                  "specification: [convention][orientation]"));
    if (!config["convention"]["position"]["type"])
      throw(std::invalid_argument("Please specify position convention type: "
                                  "[convention][position][type]"));

    if (config["convention"]["position"]["type"].as<std::string>() == "FULL")
      _is_position = true;
    else if (config["convention"]["position"]["type"].as<std::string>() ==
             "NONE")
      _is_position = false;
    else
      throw(std::invalid_argument(
          std::string("Unknown position convention type: "
                      "[convention][position][type]: ") +
          config["convention"]["position"]["type"].as<std::string>()));

    if (!_is_position)
      _linear_state = _offset_position;

    if (config["initialize"] && config["initialize"].as<bool>())
    {
      //        bool started = false;
      int tries = 0, max_tries = 10;

      //        ros::Subscriber init_sub = _node.subscribe<Message>(
      //            config["source"].as<std::string>(), 1,
      //            boost::bind(&RosOperationalEuler::_init, this, _1,
      //            &started));
      ros::Rate rate(10);

      while (ros::ok() && !_initialized && tries < max_tries)
      {
        ros::spinOnce();
        std::cout << "Waiting for feedback from " +
                         config["source"].as<std::string>() +
                         " to initialized. Try " << std::to_string(tries + 1)
                  << "/" << max_tries << std::endl;
        tries++;
        rate.sleep();
      }

      if (_initialized)
        std::cout << "Feedback from " + config["source"].as<std::string>() +
                         " initialized" << std::endl;
      else
        throw(std::invalid_argument("Couldn't initialize a callback"));
    }
    else
      std::cout << "Feedback from " + config["source"].as<std::string>() +
                       " doesn't need initialization" << std::endl;

    std::cout << "Loaded ROS operational feedback " << config["name"]
              << std::endl;
  }

  virtual ~RosOperationalEuler() {}

  virtual bool initialized() { return _initialized; }

  virtual bool get() { return true; }

  void get(const MessagePtr& msg)
  {
    _initialized = true;

    if(!_size) return;
    if (_position)
    {
      // this works for the QUATERNION:HAMILTONIAN CONVENTION
      _orientation.x() = msg->pose[_ref].orientation.x;
      _orientation.y() = msg->pose[_ref].orientation.y;
      _orientation.z() = msg->pose[_ref].orientation.z;
      _orientation.w() = -msg->pose[_ref].orientation.w;

      if (_is_position)
        _linear_state << msg->pose[_ref].position.x, msg->pose[_ref].position.y,
            msg->pose[_ref].position.z;

      getPosition(_orientation, _linear_state);
    }

    if (_velocity)
    {
      _full << msg->twist[_ref].linear.x, msg->twist[_ref].linear.y, msg->twist[_ref].linear.z,
          msg->twist[_ref].angular.x, msg->twist[_ref].angular.y,
          msg->twist[_ref].angular.z;
      getVelocity(_full);
    }

  }

protected:
  ros::NodeHandle _node;
  ros::Subscriber _state_sub;
  mwoibn::Vector3 _linear_state;

  mwoibn::Quaternion _orientation;
  bool _initialized = false, _is_position, _is_orientation;
  int _ref;
};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
