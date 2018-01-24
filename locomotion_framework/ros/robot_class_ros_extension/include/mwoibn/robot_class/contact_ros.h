#ifndef ROBOT_CLASS_CONTACT_ROS_H
#define ROBOT_CLASS_CONTACT_ROS_H

#include <vector>
#include <ros/ros.h>
#include <gazebo_msgs/ContactsState.h>
#include <mwoibn/robot_class/robot_class.h>

namespace mwoibn
{
namespace robot_class
{

template <typename Contact> class ContactRos : public Contact
{

public:
  ContactRos(Contact&& contact, std::string topic) : Contact(std::move(contact))
  {
    _subscribe = _node.subscribe<gazebo_msgs::ContactsState>(
        topic, 1, &mwoibn::robot_class::ContactRos<Contact>::_stateCallback, this);
  }
  ContactRos(Contact* contact, std::string topic) : Contact(std::move(*contact))
  {
    _subscribe = _node.subscribe<gazebo_msgs::ContactsState>(
        topic, 1, &mwoibn::robot_class::ContactRos<Contact>::_stateCallback, this);
  }
  ContactRos(Contact& contact, std::string topic) : Contact(contact)
  {
    _subscribe = _node.subscribe<gazebo_msgs::ContactsState>(
        topic, 1, &mwoibn::robot_class::ContactRos<Contact>::_stateCallback, this);
  }

  ContactRos(Contact&& contact, YAML::Node config) : Contact(std::move(contact))
  {
    if(!config["topic"] || !config["settings"]["feedback"].as<bool>()) return;

    _subscribe = _node.subscribe<gazebo_msgs::ContactsState>(
        config["topic"].as<std::string>(), 1, &mwoibn::robot_class::ContactRos<Contact>::_stateCallback, this);
  }
  ContactRos(Contact* contact, YAML::Node config) : Contact(std::move(*contact))
  {
     if(!config["topic"] || !config["settings"]["feedback"].as<bool>()) return;

     _subscribe = _node.subscribe<gazebo_msgs::ContactsState>(
         config["topic"].as<std::string>(), 1, &mwoibn::robot_class::ContactRos<Contact>::_stateCallback, this);

  }
  ContactRos(Contact& contact, YAML::Node config) : Contact(contact)
  {
    if(!config["topic"] || !config["settings"]["feedback"].as<bool>()) return;

    _subscribe = _node.subscribe<gazebo_msgs::ContactsState>(
        config["topic"].as<std::string>(), 1, &mwoibn::robot_class::ContactRos<Contact>::_stateCallback, this);

  }

  ContactRos(Contact&& contact) : Contact(std::move(contact))
  {  }
  ContactRos(Contact* contact) : Contact(std::move(*contact))
  {  }
  ContactRos(Contact& contact) : Contact(contact)
  {  }

  virtual ~ContactRos() {}

protected:
  ros::NodeHandle _node;
//  void _initCallback();
  void _stateCallback(const gazebo_msgs::ContactsState::ConstPtr& msg)
  {
    if (msg->states.size())
      Contact::activate();

    else
      Contact::deactivate();
  }
  ros::Subscriber _subscribe;
};
}
}
#endif // ROBOT_CLASS_CONTACT_ROS_H
