#ifndef __MWOIBN__COMMUNICATION_MODULES__ROS_POINT_FEEDBACK_H
#define __MWOIBN__COMMUNICATION_MODULES__ROS_POINT_FEEDBACK_H

#include "mwoibn/communication_modules/basic_point.h"
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

namespace mwoibn
{
namespace communication_modules
{

class RosPointFeedback : public BasicPoint
{

public:
  // for now only full robot is supported for this controller
  RosPointFeedback(mwoibn::point_handling::State& point, YAML::Node config)
      : BasicPoint(point, config)
  {
    if (!config["source"])
      throw(std::invalid_argument("Missing required parameter: source"));

    _state_sub =
        _node.subscribe<geometry_msgs::Twist>(config["source"].as<std::string>(), 1,
                                 boost::bind(&RosPointFeedback::get, this, _1));

   if(!config["interface"]) throw(std::invalid_argument("Missing required parameter: interface"));

   if(config["interface"].as<std::string>() == "world") _world = true;
   else if(config["interface"].as<std::string>() == "fixed") _world = false;
   else throw(std::invalid_argument("Shared point get: Unknow 'interface': " + config["interface"].as<std::string>()));


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

  RosPointFeedback(RosPointFeedback& other)
      : BasicPoint(other),   _node(other._node), _world(other._world), _state(other._state)
  {
    std::string topic = other._state_sub.getTopic();
    other._state_sub.shutdown();

        _state_sub =
            _node.subscribe<geometry_msgs::Twist>(topic, 1,
                                     boost::bind(&RosPointFeedback::get, this, _1));
   }


  RosPointFeedback(RosPointFeedback&& other)
  : BasicPoint(other),   _node(other._node), _world(other._world), _state(other._state)
  {
    std::string topic = other._state_sub.getTopic();
    other._state_sub.shutdown();

    _state_sub =
        _node.subscribe<geometry_msgs::Twist>(topic, 1,
                                 boost::bind(&RosPointFeedback::get, this, _1));

   }


  virtual ~RosPointFeedback() {}
  virtual bool initialized() { return _initialized; }
  virtual bool run() { return true; }

  void get(const geometry_msgs::Twist::ConstPtr& msg)
  {

      //std::cout << "ros contact feedback" << std::endl;
    //if (!_initialized)
      //_initFilters(msg);
      _state << msg->angular.x, msg->angular.y, msg->angular.z, msg->linear.x, msg->linear.y, msg->linear.z;
      if(_world)
      _point.setWorld(_state);
      else
      _point.setFixed(_state);


    _initialized = true;
  }

protected:
  ros::NodeHandle _node;
  ros::Subscriber _state_sub;
  bool _world;
  mwoibn::Vector6 _state;
};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
