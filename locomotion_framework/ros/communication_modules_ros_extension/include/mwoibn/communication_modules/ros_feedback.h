#ifndef COMMUNICATION_MODULES_ROS_FEEDBACK_H
#define COMMUNICATION_MODULES_ROS_FEEDBACK_H

#include "mwoibn/communication_modules/basic_feedback.h"
#include "ros/ros.h"
#include <custom_messages/CustomCmnd.h>

namespace mwoibn
{
namespace communication_modules
{
template <typename Message, typename MessagePtr>
class RosFeedback : public BasicFeedback
{

public:
  // for now only full robot is supported for this controller
  RosFeedback(mwoibn::robot_class::State& command,
              mwoibn::robot_class::BiMap& map, YAML::Node config)
      : BasicFeedback(command, map, config){
        _init(config);
      }

      RosFeedback(mwoibn::robot_class::State& command,
                  mwoibn::robot_class::BiMap&& map, YAML::Node config)
          : BasicFeedback(command, map, config){
            _init(config);
          }

  RosFeedback(RosFeedback& other)
      : BasicFeedback(other),   _node(other._node), _filtered(other._filtered)
  {
    std::string topic = other._state_sub.getTopic();
    other._state_sub.shutdown();
    _state_sub =
        _node.subscribe<Message>(topic, 1,
                                 boost::bind(&RosFeedback::get, this, _1));
  }

  RosFeedback(RosFeedback&& other)
      : BasicFeedback(other),   _node(other._node), _filtered(other._filtered)
  {
    std::string topic = other._state_sub.getTopic();
    other._state_sub.shutdown();
    _state_sub =
        _node.subscribe<Message>(topic, 1,
                                 boost::bind(&RosFeedback::get, this, _1));
  }

  virtual ~RosFeedback() {}
  virtual bool initialized() { return _initialized; }
  virtual bool run() { return true; }

  void get(const MessagePtr& msg)
  {
    if (!_initialized)
      _initFilters(msg);

    if (_position)
      _command.position.set(msg->position, _map.reversed());
    if (_velocity)
      _command.velocity.set(msg->velocity, _map.reversed());
    if (_torque)
      _command.torque.set(msg->effort, _map.reversed());

    _initialized = true;
  }


protected:
  ros::NodeHandle _node;
  ros::Subscriber _state_sub;

  mwoibn::VectorN _filtered;

  void _initFilters(const MessagePtr& msg)
  {
    if (!_filter)
      return;

    if (_position)
      _position_filter_ptr->reset(msg->position);
    if (_velocity)
      _velocity_filter_ptr->reset(msg->velocity);
    if (_torque)
      _torque_filter_ptr->reset(msg->effort);
  }

void _init(YAML::Node config){
  {
    if (!config["source"])
      throw(std::invalid_argument("Missing required parameter: source"));

    _state_sub =
        _node.subscribe<Message>(config["source"].as<std::string>(), 1,
                                 boost::bind(&RosFeedback::get, this, _1));

    if (config["initialize"] && config["initialize"].as<bool>())
    {
      //      bool started = false;
      int tries = 0, max_tries = 100;

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
    //    std::cout << config["source"].as<std::string>() << std::endl;

    if (_filter)
    {
      if (!config["rate"])
        throw(std::invalid_argument(
            "Missing parameter required to initialize filters : rate"));

      _filtered.setZero(_dofs);
      std::cout << _dofs << std::endl;

      if (_position)
        _position_filter_ptr->computeCoeffs(config["rate"].as<double>());
      if (_velocity)
        _velocity_filter_ptr->computeCoeffs(config["rate"].as<double>());
      if (_torque)
        _torque_filter_ptr->computeCoeffs(config["rate"].as<double>());
    }

    std::cout << "Loaded ROS feedback " << config["name"] << std::endl;
  }

}

};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
