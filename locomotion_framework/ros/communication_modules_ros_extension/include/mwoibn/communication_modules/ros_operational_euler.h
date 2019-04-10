#ifndef COMMUNICATION_MODULES_ROS_OPERATIONAL_EULER_H
#define COMMUNICATION_MODULES_ROS_OPERATIONAL_EULER_H

#include "mwoibn/communication_modules/basic_operational_euler.h"
#include "ros/ros.h"
#include <custom_messages/CustomCmnd.h>

namespace mwoibn
{
namespace communication_modules
{
template <typename Message, typename MessagePtr>
class RosOperationalEuler : public BasicOperationalEuler
{

public:
  // for now only full robot is supported for this controller
  RosOperationalEuler(mwoibn::robot_class::State& command,
                      mwoibn::robot_class::BiMap& map, YAML::Node config)
      : BasicOperationalEuler(command, map, config){
        _init(config);
      }

      RosOperationalEuler(mwoibn::robot_class::State& command,
                          mwoibn::robot_class::BiMap&& map, YAML::Node config)
          : BasicOperationalEuler(command, map, config){
            _init(config);
          }

  RosOperationalEuler(RosOperationalEuler& other)
      : BasicOperationalEuler(other),   _node(other._node),  _linear_state(other._linear_state),
      _ref_link(other._ref_link),  _orientation(other._orientation), _is_position(other._is_position), _is_orientation(other._is_orientation), _ref(other._ref)
  {
    std::string topic = other._state_sub.getTopic();
    other._state_sub.shutdown();

    _state_sub = _node.subscribe<Message>( topic, 1,
        boost::bind(&RosOperationalEuler::get, this, _1));


   }


  RosOperationalEuler(RosOperationalEuler&& other)
  : BasicOperationalEuler(other),   _node(other._node), _linear_state(other._linear_state),
  _ref_link(other._ref_link),  _orientation(other._orientation), _is_position(other._is_position), _is_orientation(other._is_orientation), _ref(other._ref)
  {

    std::string topic = other._state_sub.getTopic();
    other._state_sub.shutdown();

    _state_sub = _node.subscribe<Message>( topic, 1,
        boost::bind(&RosOperationalEuler::get, this, _1));

   }

  virtual ~RosOperationalEuler() {}

  virtual bool initialized() { return _initialized; }

  virtual bool run() { return true; }

  void get(const MessagePtr& msg)
  {
    if (!_initialized)
    {

      for (int i = 0; i < msg->name.size(); i++)
      {
        if (msg->name[i] != _ref_link)
          continue;

        _ref = i;
        std::cout << "Found a floating base link " << _ref_link << " " << _ref << "." << std::endl;

        break;
      }
      if(_ref == mwoibn::NON_EXISTING)
        std::cout << "Reference frame " << _ref_link << " has not been defined in the recieved message." << std::endl;
      else{
      _initialized = true;
      return;
      }
    }

    if (!_size)
      return;
    if (_position)
    {
      // this works for the QUATERNION:HAMILTONIAN CONVENTION
      _orientation.x() = msg->pose[_ref].orientation.x;
      _orientation.y() = msg->pose[_ref].orientation.y;
      _orientation.z() = msg->pose[_ref].orientation.z;
      _orientation.w() = msg->pose[_ref].orientation.w;

      // std::cout << "orientation\t" << _orientation << std::endl;
      if (_is_position)
        _linear_state << msg->pose[_ref].position.x, msg->pose[_ref].position.y,
            msg->pose[_ref].position.z;

      getPosition(_orientation, _linear_state);
    }

    if (_velocity)
    {

      // this is the angular velocity not euler angles



      mwoibn::Matrix3 to_euler;
      double x = _command.position.get()[_map_dofs[3]];
      double y = _command.position.get()[_map_dofs[4]];
      double z = _command.position.get()[_map_dofs[5]];

      to_euler << 1, std::sin(x)*std::sin(y)/std::cos(y), -std::cos(x)*std::sin(y)/std::cos(y),
                   0, std::cos(x), std::sin(x),
                   0, -std::sin(x)/std::cos(y), std::cos(x)/std::cos(y);

     mwoibn::Vector3 angular;
     angular << msg->twist[_ref].angular.x, msg->twist[_ref].angular.y,
              msg->twist[_ref].angular.z;

    // std::cout << "angular velocity\t" << angular.transpose() << std::endl;
    _full.tail<3>().noalias() = to_euler*angular;
      _full[0] = msg->twist[_ref].linear.x;
      _full[1] = msg->twist[_ref].linear.y;
      _full[2] = msg->twist[_ref].linear.z;

      // std::cout << "euler vel\t" << _full.tail<3>().transpose() << std::endl;
      getVelocity(_full);
    }
  }

protected:
  ros::NodeHandle _node;
  ros::Subscriber _state_sub;
  mwoibn::Vector3 _linear_state;

  std::string _ref_link;
  mwoibn::Quaternion _orientation;
  bool _is_position, _is_orientation;
  int _ref = mwoibn::NON_EXISTING;


  void _init(YAML::Node config){

      if (!config["source"])
        throw(std::invalid_argument("Missing required parameter: source"));

      _state_sub = _node.subscribe<Message>(
          config["source"].as<std::string>(), 1,
          boost::bind(&RosOperationalEuler::get, this, _1));

      if (!config["reference"])
        throw(std::invalid_argument("Couldn't find an argument [reference]"));

      _ref_link = config["reference"].as<std::string>();

      if (!config["convention"])
        throw(std::invalid_argument(
            "Couldn't find a convention specification: [convention]"));
      if (!config["convention"]["position"])
        throw(std::invalid_argument("Couldn't find a position convention "
                                    "specification: [convention][position]"));
      if (!config["convention"]["orientation"])
        throw(std::invalid_argument("Couldn't find an orientation convention "
                                    "specification: [convention][orientation]"));
      if (!config["convention"]["position"]["type"])
        throw(std::invalid_argument("Please specify position convention type: "
                                    "[convention][position][type]"));

      if (config["convention"]["position"]["type"].as<std::string>() == "FULL")
        _is_position = true;
      else if (config["convention"]["position"]["type"].as<std::string>() ==
               "NONE")
        _is_position = false;
      else
        throw(std::invalid_argument(
            std::string("Unknown position convention type: "
                        "[convention][position][type]: ") +
            config["convention"]["position"]["type"].as<std::string>()));

      if (!_is_position)
        _linear_state = _offset_position;

      if (config["initialize"] && config["initialize"].as<bool>())
      {
        //        bool started = false;
        int tries = 0, max_tries = 10;

        //        ros::Subscriber init_sub = _node.subscribe<Message>(
        //            config["source"].as<std::string>(), 1,
        //            boost::bind(&RosOperationalEuler::_init, this, _1,
        //            &started));
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

      std::cout << "Loaded ROS operational feedback " << config["name"]
                << std::endl;

  }
};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
