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
              mwoibn::robot_class::BiMap map, YAML::Node config)
      : BasicFeedback(command, map, config)
  {
    if (!config["source"])
      throw(std::invalid_argument("Missing required parameter: source"));

    _state_sub =
        _node.subscribe<Message>(config["source"].as<std::string>(), 1,
                                 boost::bind(&RosFeedback::get, this, _1));

    if (config["initialize"] && config["initialize"].as<bool>())
    {
      //      bool started = false;
      int tries = 0, max_tries = 10;

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

    std::cout << "Loaded ROS feedback " << config["name"] << std::endl;
  }
  virtual ~RosFeedback() {}
  virtual bool initialized() { return _initialized; }
  virtual bool get() { return true; }

  void get(const MessagePtr& msg)
  {
    if (_position)
      _command.set(msg->position, _map.reversed(),
                   robot_class::INTERFACE::POSITION);
    if (_velocity)
      _command.set(msg->velocity, _map.reversed(),
                   robot_class::INTERFACE::VELOCITY);
    if (_torque)
      _command.set(msg->effort, _map.reversed(),
                   robot_class::INTERFACE::TORQUE);

    _initialized = true;
  }

protected:
  ros::NodeHandle _node;
  ros::Subscriber _state_sub;

  bool _initialized = false;
};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
