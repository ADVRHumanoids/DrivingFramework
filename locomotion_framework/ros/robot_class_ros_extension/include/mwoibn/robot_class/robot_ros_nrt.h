#ifndef ROBOT_CLASS_ROBOT_ROS_NRT_H
#define ROBOT_CLASS_ROBOT_ROS_NRT_H

#include "mwoibn/robot_class/robot_ros.h"
#include <custom_messages/CustomCmnd.h>

#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>
#include <climits>

#include "mwoibn/communication_modules/all.h"
#include "mwoibn/robot_class/contact_ros.h"

namespace mwoibn
{
namespace robot_class
{

//! Implementation of the robot_class for ROS with ros_control controllers
class RobotRosNRT : public RobotRos
{

public:
  RobotRosNRT(std::string config_file, std::string config_name,
              std::string secondary_file = "");

  virtual ~RobotRosNRT() {}

  virtual bool send()
  {
    bool success = Robot::send();
    if (!_spined)
    {
      ros::spinOnce();
      _spined = true;
    }
    return success;
  }

  virtual bool get()
  {
    if (!_spined)
    {
      ros::spinOnce();
      _spined = true;
    }
    return Robot::get();
  }

  //  virtual void wait(){
  //    _spin = false;
  //    _rate_ptr->sleep();
  //  }

  template <typename MessagePtr>
  void _initMappingCallback(const MessagePtr& msg, bool* valid,
                            std::string name)
  {

    std::vector<std::string> names = msg->name;

    biMaps().add(makeBiMap(getLinks(msg->name), name, names));
    *valid = true;
  }

  template <typename Message, typename MeessagePtr>
  bool initMapping(std::string topic, std::string name, int tries = 10)
  {

    bool valid = false;
    int count = 0;

    ros::Subscriber init_sub = _node.template subscribe<Message>(
        topic, 1,
        boost::bind(
            &mwoibn::robot_class::RobotRosNRT::template _initMappingCallback<
                MeessagePtr>,
            this, _1, &valid, name));

    while (!valid && ros::ok() && (count < tries))
    {

#ifdef LOGGER
      LOG_INFO << "waiting for callback" << std::endl;
#endif
      count++;
      wait();
      get();
      // update();
    }

    init_sub.shutdown();

    return valid;
  }



  static bool loadJointSpaceFeedback(YAML::Node config,
                                     Feedbacks& external_feedbacks,
                                     State& external_state, BiMap external_map);
  static bool loadOperationalSpaceFeedback(YAML::Node config,
                                           Feedbacks& external_feedbacks,
                                           State& external_state,
                                           BiMap external_map);
  static bool loadRosControllers(YAML::Node config,
                                 Controllers& external_controllers,
                                 State& external_state, BiMap external_map);


protected:
  virtual void _loadFeedbacks(YAML::Node config);

  void _loadControllers(YAML::Node config);

  virtual void _loadContacts(YAML::Node contacts_config);
  virtual void _loadMapFromTopic(YAML::Node config);
  virtual void _loadMap(YAML::Node config);

};
} // namespace package
} // namespace library
#endif
