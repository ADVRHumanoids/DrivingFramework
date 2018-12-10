#ifndef __MWOIBN__COMMUNICATION_MODULES__ROS_CONTACT_SIMULATION_H
#define __MWOIBN__COMMUNICATION_MODULES__ROS_CONTACT_SIMULATION_H

#include "mwoibn/communication_modules/basic_point.h"
#include "ros/ros.h"
#include <gazebo_msgs/ContactsState.h>


namespace mwoibn
{
namespace communication_modules
{

class RosContactSimulation : public BasicPoint
{

public:
  // for now only full robot is supported for this controller
  RosContactSimulation(mwoibn::point_handling::Wrench& point, YAML::Node config)
      : BasicPoint(point, config)
  {

          if(!config["topic"]) return;
          if(!config["message"]) return;


          if(config["message"].as<std::string>() == "gazebo_msgs::ContactsState")
             _subscribe = _node.subscribe<gazebo_msgs::ContactsState>(
             config["topic"].as<std::string>(), 1, &mwoibn::communication_modules::RosContactSimulation::get, this);



                   //      bool started = false;
           int tries = 0, max_tries = 100;

             ros::Rate rate(10);

           while (ros::ok() && !_initialized && tries < max_tries)
           {
               ros::spinOnce();
               std::cout << "Waiting for feedback from " +
                              config["topic"].as<std::string>() +
                              " to initialized. Try " << std::to_string(tries + 1)
                         << "/" << max_tries << std::endl;
               tries++;
               rate.sleep();
           }
           if (_initialized)
                 std::cout << "Feedback from " + config["topic"].as<std::string>() +
                                " initialized" << std::endl;
           else
               throw(std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": Couldn't initialize a callback")));

        std::cout << "Loaded ROS contact simulation feedback " << config["name"] << std::endl;
  }

  RosContactSimulation(RosContactSimulation& other)
      : BasicPoint(other), _node(other._node), _state(other._state)
  {
    std::string topic = other._subscribe.getTopic();
    other._subscribe.shutdown();
    _subscribe = _node.subscribe<gazebo_msgs::ContactsState>(
    topic, 1, &mwoibn::communication_modules::RosContactSimulation::get, this);

  }

  RosContactSimulation(RosContactSimulation&& other)
      : BasicPoint(other), _node(other._node), _state(other._state)
  {
    std::string topic = other._subscribe.getTopic();
    other._subscribe.shutdown();
    _subscribe = _node.subscribe<gazebo_msgs::ContactsState>(
    topic, 1, &mwoibn::communication_modules::RosContactSimulation::get, this);
   }
  virtual ~RosContactSimulation() {}
  virtual bool initialized() { return _initialized; }
  virtual bool run() { return true; }

  void get(const gazebo_msgs::ContactsState::ConstPtr& msg)
  {
      if(msg->states.size() && msg->states[0].wrenches.size()){

          _state <<  msg->states[0].wrenches[0].torque.x, msg->states[0].wrenches[0].torque.y, msg->states[0].wrenches[0].torque.z, msg->states[0].wrenches[0].force.x, msg->states[0].wrenches[0].force.y, msg->states[0].wrenches[0].force.z;
          _point.setFixed(_state);

      }

    _initialized = true;
  }

protected:
  ros::NodeHandle _node;
  ros::Subscriber _subscribe;

  mwoibn::Vector6 _state;

};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
