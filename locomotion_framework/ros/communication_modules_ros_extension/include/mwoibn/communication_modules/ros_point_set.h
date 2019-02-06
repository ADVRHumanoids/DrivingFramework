#ifndef __MWOIBN__COMMUNICATION_MODULES__ROS_POINT_SET_H
#define __MWOIBN__COMMUNICATION_MODULES__ROS_POINT_SET_H

#include "mwoibn/communication_modules/basic_point.h"
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

namespace mwoibn
{
namespace communication_modules
{

class RosPointSet : public BasicPoint
{

public:
  // for now only full robot is supported for this controller
  RosPointSet(mwoibn::point_handling::State& point, YAML::Node config)
      : BasicPoint(point, config)
  {
    if (!config["topic"])
      throw(std::invalid_argument("Missing required parameter: topic"));

      _command_pub = _node.advertise<geometry_msgs::Twist>(config["topic"].as<std::string>(), 1);

      if(!config["interface"]) throw(std::invalid_argument("Missing required parameter: interface"));

      if(config["interface"].as<std::string>() == "world") _world = true;
      else if(config["interface"].as<std::string>() == "fixed") _world = false;
      else throw(std::invalid_argument("Shared point get: Unknow 'interface': " + config["interface"].as<std::string>()));



    std::cout << "Loaded ROS point setter " << config["name"] << std::endl;
  }

  RosPointSet(RosPointSet& other)
      : BasicPoint(other),   _node(other._node), _des_q(other._des_q)
  {
    std::string topic = other._command_pub.getTopic();
    other._command_pub.shutdown();
    _command_pub = _node.advertise<geometry_msgs::Twist>(topic, 1);

  }


  RosPointSet(RosPointSet&& other)
  : BasicPoint(other),   _node(other._node), _des_q(other._des_q)
  {
    std::string topic = other._command_pub.getTopic();
    other._command_pub.shutdown();
    _command_pub = _node.advertise<geometry_msgs::Twist>(topic, 1);
  }


  virtual ~RosPointSet() {}
  virtual bool initialized() { return _initialized; }
  virtual bool run() {
    if(_world)
      _state = _point.getWorld();
    else
      _state = _point.getFixed();

      _des_q.angular.x = _state[0];
      _des_q.angular.y = _state[1];
      _des_q.angular.z = _state[2];
      _des_q.linear.x = _state[3];
      _des_q.linear.y = _state[4];
      _des_q.linear.z = _state[5];

    _initialized = true;

    _command_pub.publish(_des_q);

    return true;
  }


protected:
  mwoibn::Vector6 _state;
  ros::NodeHandle _node;
  geometry_msgs::Twist _des_q;
  ros::Publisher _command_pub;
 bool _world;

};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
