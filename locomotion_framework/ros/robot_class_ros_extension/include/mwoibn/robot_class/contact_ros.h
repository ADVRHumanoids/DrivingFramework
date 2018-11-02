#ifndef __MWOIBN__ROBOT_CLASS__CONTACT_ROS_H
#define __MWOIBN__ROBOT_CLASS__CONTACT_ROS_H

#include <vector>
#include <ros/ros.h>
#include <gazebo_msgs/ContactsState.h>
#include "mwoibn/common/all.h"
#include <std_srvs/SetBool.h>
#include "mwoibn/communication_modules/ros_point_feeback.h"

namespace mwoibn
{
namespace robot_class
{

template <typename Contact> class ContactRos : public Contact
{

public:
/*ContactRos(Contact&& contact, std::string topic, std::string type) : Contact(std::move(contact))
{
    _initCallbacks(topic, type);
}
ContactRos(Contact* contact, std::string topic, std::string type) : Contact(std::move(*contact))
{
    _initCallbacks(topic, type);
}

ContactRos(Contact& contact, std::string topic, std::string type) : Contact(contact)
{
      _initCallbacks(topic, type);
}
*/
ContactRos(Contact&& contact, YAML::Node config) : Contact(std::move(contact))
{
        _f.setZero(6);
        _p.setZero(3);
        _initCallbacks(config);
}

ContactRos(Contact* contact, YAML::Node config) : Contact(std::move(*contact))
{
        _f.setZero(6);
        _p.setZero(3);
        _initCallbacks(config);
}

ContactRos(Contact& contact, YAML::Node config) : Contact(contact)
{
        _f.setZero(6);
        _p.setZero(3);
        _initCallbacks(config);
}

void setCallback(){_callback = true;}
void stopCallback(){_callback = false;}

ContactRos(Contact&& contact) : Contact(std::move(contact))
{
}
ContactRos(Contact* contact) : Contact(std::move(*contact))
{
}
ContactRos(Contact& contact) : Contact(contact)
{
}

virtual ~ContactRos() {
}

protected:
ros::NodeHandle _node;
bool _callback = false;
mwoibn::VectorN _f, _p;

std::unique_ptr<mwoibn::communication_modules::RosPointFeedback> _subsrciber;
// (
//         new mwoibn::communication_modules::RosFeedback<
//                 custom_messages::CustomCmnd,
//                 custom_messages::CustomCmnd::ConstPtr>(external_state,
//                                                        external_map, config))

//  void direct gazebo callback (simulated state);
void _stateCallback(const gazebo_msgs::ContactsState::ConstPtr& msg)
{
  if(msg->states.size() && msg->states[0].wrenches.size()){

      _f <<  msg->states[0].wrenches[0].torque.x, msg->states[0].wrenches[0].torque.y, msg->states[0].wrenches[0].torque.z, msg->states[0].wrenches[0].force.x, msg->states[0].wrenches[0].force.y, msg->states[0].wrenches[0].force.z;
      Contact::_wrench.setFixed(_f);

      // full with position

      //_p << msg->states[0].contact_positions[0].x, msg->states[0].contact_positions[0].y, msg->states[0].contact_positions[0].z;
      //Contact::_frame.setLinearWorld(_p);
  }

  if (!_callback) return;

  if (msg->states.size())
      Contact::activate();
  else
      Contact::deactivate();
}
 // I will also need the XBot implementation


bool _activationCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res){

  if (req.data) setCallback();
  else stopCallback();

  return true;
}

void _initCallbacks(YAML::Node config){

  if(!config["topic"]) return;
  if(!config["settings"]["feedback"].as<bool>()) return;

  //for(auto entry: config) std::cout << entry.first << std::endl;
  if(!config["message"]) return;

//  if(!config["topic"] || !config["settings"]["feedback"].as<bool>() || !config["message"]) return;

  if(config["message"].as<std::string>() == "gazebo_msgs::ContactsState")
     _subscribe = _node.subscribe<gazebo_msgs::ContactsState>(
     config["topic"].as<std::string>(), 1, &mwoibn::robot_class::ContactRos<Contact>::_stateCallback, this);
  else if(config["message"].as<std::string>() == "geometry_msgs::Twist"){
      config["source"] = config["topic"];
     _subsrciber.reset(new mwoibn::communication_modules::RosPointFeedback(Contact::_wrench, config));
   }
  else
     throw std::invalid_argument(std::string("Ros Contact: unknown feedback type: ") + config["message"].as<std::string>());


  // _service = _node.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
  //                        config["topic"]["type"].as<std::string>(), boost::bind(&mwoibn::robot_class::ContactRos<Contact>::_activationCallback, this, _1, _2));


}

ros::Subscriber _subscribe;
ros::ServiceServer _service;
};
}
}
#endif // ROBOT_CLASS_CONTACT_ROS_H
