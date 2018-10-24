#ifndef ROBOT_CLASS_ROBOT_ROS_H
#define ROBOT_CLASS_ROBOT_ROS_H

#include "mwoibn/robot_class/robot.h"

#include <ros/ros.h>

#include <urdf/model.h>
#include <srdfdom/model.h>

namespace mwoibn
{
namespace robot_class
{

//! Implementation of the robot_class for ROS
/**
 * Automatically initializes RBDL model from given ros parameter.
 *
 */
class RobotRos : public Robot
{

public:
  /**
   * \param[in] is_static information about the model type is required as there
   * are inconsistencies in the output from certain RBDL functions between two
   * types of models
   * \param[in] topic name of the ros parameter comprising the robot
   * description, if no value given dafualt "/robot_description" is used
   */
//  RobotRos(std::string urdf_topic = "/robot_description",
//           std::string srdf_topic = "", bool loadActuationConfiguration = true,
//           bool loadContactsConfiguration = true);
  /**
   * @brief Constructor from a parameter file
   */
  RobotRos(std::string config_file, std::string config_name, std::string secondary_file = "");
  RobotRos(YAML::Node full_config, std::string config_name);

  virtual ~RobotRos() { _node.shutdown(); }
  virtual bool isRunning() { return ros::ok(); }

  virtual void wait(){
    _spined = false;
    _rate_ptr->sleep();
  }

  virtual double rate() { return _rate_ptr->expectedCycleTime().toSec(); }


protected:
  //! Keeps the ros_node handler
  ros::NodeHandle _node;

  RobotRos(){}

  int _base_ref = -1;

  YAML::Node _init(YAML::Node config, std::string config_name);
  std::string _readUrdf(YAML::Node config);
  std::string _readSrdf(YAML::Node config);

  virtual void _readActuators(YAML::Node config);
  virtual void _readContacts(YAML::Node config);

  std::string _readConfigString(YAML::Node config, std::string name);

  virtual bool _initUrdf(std::string& urdf_description, urdf::Model& urdf);
  virtual srdf::Model _initSrdf(std::string& srdf_description, urdf::Model& urdf);

  virtual void _initModel(bool is_static, const std::string& source, RigidBodyDynamics::Model& model);
  bool _spined = false;
  std::unique_ptr<ros::Rate> _rate_ptr;

};
} // namespace package
} // namespace library
#endif
