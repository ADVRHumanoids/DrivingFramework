#ifndef __MWOIBN__COMMUNICATION_MODULES__ROS_POINT_FEEDBACK_H
#define __MWOIBN__COMMUNICATION_MODULES__ROS_POINT_FEEDBACK_H

#include "mwoibn/communication_modules/point_feedback.h"
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

namespace mwoibn
{
namespace communication_modules
{

class RosPointFeedback : public PointFeedback
{

public:
  // for now only full robot is supported for this controller
  RosPointFeedback(mwoibn::point_handling::Wrench& point, YAML::Node config)
      : PointFeedback(point, config)
  {
    if (!config["source"])
      throw(std::invalid_argument("Missing required parameter: source"));

    _state_sub =
        _node.subscribe<geometry_msgs::Twist>(config["source"].as<std::string>(), 1,
                                 boost::bind(&RosPointFeedback::get, this, _1));

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
/*
    if (_filter)
    {
      if (!config["rate"])
        throw(std::invalid_argument(
            "Missing parameter required to initialize filters : rate"));

      _filtered.setZero(3);

      _linear_filter_ptr->computeCoeffs(config["rate"].as<double>());
      _angular_filter_ptr->computeCoeffs(config["rate"].as<double>());
    }
*/
    std::cout << "Loaded ROS feedback " << config["name"] << std::endl;
  }
  virtual ~RosPointFeedback() {}
  virtual bool initialized() { return _initialized; }
  virtual bool get() { return true; }

  void get(const geometry_msgs::Twist::ConstPtr& msg)
  {

      //std::cout << "ros contact feedback" << std::endl;
    //if (!_initialized)
      //_initFilters(msg);
      _state << msg->angular.x, msg->angular.y, msg->angular.z, msg->linear.x, msg->linear.y, msg->linear.z,
      _wrench.setFixed(_state);


    _initialized = true;
  }
  //
  // virtual bool raw(mwoibn::VectorN& _raw,
  //                  mwoibn::robot_class::INTERFACE interface)
  // {
  //
  //   _is_raw = true;
  //
  //   if (!_initialized)
  //     return false;
  //   if (!is(interface))
  //     return false;
  //
  //   _raw_interface = interface;
  //
  //   int tries = 0, max_tries = 10;
  //   ros::Rate rate(10);
  //
  //   while (ros::ok() && _is_raw && tries < max_tries)
  //   {
  //     ros::spinOnce();
  //     std::cout << "Waiting for raw feedback. Try " << std::to_string(tries + 1)
  //               << "/" << max_tries << std::endl;
  //     tries++;
  //     rate.sleep();
  //   }
  //   if (!_is_raw)
  //   {
  //     _raw = _raw_keep;
  //     std::cout << "Received raw feedback." << std::endl;
  //     _is_raw = false;
  //     return true;
  //   }
  //   else
  //   {
  //     std::cout << "Couldn't return raw feedback." << std::endl;
  //     _is_raw = false;
  //     return false;
  //   }
  // }

protected:
  ros::NodeHandle _node;
  ros::Subscriber _state_sub;

  //mwoibn::VectorN _raw_keep, _filtered;
  mwoibn::Vector6 _state;
  //bool _is_raw;
  bool _initialized = false;
/*
  void _initFilters(const MessagePtr& msg)
  {
    _linear_filter_ptr->reset(msg->position);
    _angular_filter_ptr->reset(msg->position);
  }
  */
};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
