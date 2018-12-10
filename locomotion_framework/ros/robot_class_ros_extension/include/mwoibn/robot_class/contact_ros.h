#ifndef __MWOIBN__ROBOT_CLASS__CONTACT_ROS_H
#define __MWOIBN__ROBOT_CLASS__CONTACT_ROS_H

#include <vector>
#include <ros/ros.h>
#include "mwoibn/common/all.h"
#include <std_srvs/SetBool.h>
#include "mwoibn/communication_modules/ros_point_feeback.h"
#include "mwoibn/communication_modules/ros_contact_simulation.h"
#include "mwoibn/communication_modules/basic_module.h"

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
        // _f.setZero(6);
        // _p.setZero(3);
        // _initCallbacks(config);
}

ContactRos(Contact* contact, YAML::Node config) : Contact(std::move(*contact))
{
        // _f.setZero(6);
        // _p.setZero(3);
        // _initCallbacks(config);
}

ContactRos(Contact& contact, YAML::Node config) : Contact(contact)
{
        // _f.setZero(6);
        // _p.setZero(3);
        // _initCallbacks(config);
}

// void setCallback(){_callback = true;}
// void stopCallback(){_callback = false;}

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

virtual std::unique_ptr<mwoibn::communication_modules::CommunicationBase> generateCallback(YAML::Node config){

  if(!config["topic"]) return std::unique_ptr<mwoibn::communication_modules::CommunicationBase>(nullptr);;
  //if(!config["settings"]["feedback"].as<bool>()) return std::unique_ptr<mwoibn::communication_modules::CommunicationBase>(nullptr);;

  if(!config["message"]) return std::unique_ptr<mwoibn::communication_modules::CommunicationBase>(nullptr);;


  if(config["message"].as<std::string>() == "gazebo_msgs::ContactsState")
       return std::unique_ptr<mwoibn::communication_modules::CommunicationBase>(new mwoibn::communication_modules::RosContactSimulation(Contact::_wrench, config));
  else if(config["message"].as<std::string>() == "geometry_msgs::Twist"){
       config["source"] = config["topic"];
    return std::unique_ptr<mwoibn::communication_modules::CommunicationBase>(new mwoibn::communication_modules::RosPointFeedback(Contact::_wrench, config));
  }

   return std::unique_ptr<mwoibn::communication_modules::CommunicationBase>(nullptr);

}

};
}
}
#endif // ROBOT_CLASS_CONTACT_ROS_H
