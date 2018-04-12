#ifndef __MGNSS_PLUGINS_ROS_BASE_H
#define __MGNSS_PLUGINS_ROS_BASE_H

#include <mgnss/modules/base.h>
#include <mwoibn/common/ros_logger.h>
#include <ros/ros.h>
//#include <custom_services/jointStateCmnd.h>

namespace mgnss
{
namespace plugins
{
class RosBase
{

public:
  RosBase(int argc, char** argv);
  ~RosBase(){}

  virtual bool init();

  virtual bool close();

  virtual void start(double time);

  virtual void stop();
  virtual void control_loop(double time);

protected:
  virtual void _setRate(double period){_controller_ptr->setRate(period);}
  virtual std::string _setName() = 0;
  virtual void _resetPrt(YAML::Node config) = 0;
  virtual void _initCallbacks() = 0;
  std::unique_ptr<mgnss::modules::Base> _controller_ptr;
  std::unique_ptr<mwoibn::robot_class::Robot> _robot_ptr;
  std::unique_ptr<mwoibn::common::Logger> _logger_ptr;

  virtual void _init(int argc, char** argv);
  std::unique_ptr<ros::NodeHandle> _n;

  bool _initialized = false, _valid = false, _rate = false;
  std::string _name = "";
  double _start;

};
}
}
#endif // RT_MY_TEST_H
