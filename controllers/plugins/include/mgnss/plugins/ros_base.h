#ifndef __MGNSS_PLUGINS_ROS_BASE_H
#define __MGNSS_PLUGINS_ROS_BASE_H

#include "mgnss/modules/base.h"
#include <mwoibn/common/ros_logger.h>
#include <ros/ros.h>
#include "mwoibn/std_utils/map.h"

#define MGNSS_REGISTER_ROS_PLUGIN_(constructor) \
extern "C" mgnss::plugins::RosBase* make() \
{ \
  return new constructor(); \
} \

namespace mgnss
{
namespace plugins
{
class RosBase
{
  typedef  std::map<std::string, std::shared_ptr<mwoibn::robot_class::Robot>> robot_map;
  typedef  MapKeyIterator<std::string, std::shared_ptr<mwoibn::robot_class::Robot>> _key_iter;
  typedef  MapValueIterator<std::string, std::shared_ptr<mwoibn::robot_class::Robot>> _val_iter;

public:
// RosBase(int argc, char** argv, std::string name): _name(name){
//    _init(argc, argv);
// }

RosBase() { }

virtual void connect(int argc, char** argv, std::string name);
virtual void connect(std::string name);

virtual ~RosBase(){
}

virtual bool init();
virtual bool init(robot_map& share_robots, std::shared_ptr<mwoibn::common::Logger>& logger_ptr, std::shared_ptr<ros::NodeHandle> n, mwoibn::communication_modules::Shared& share, std::string name);

virtual bool close();

virtual void start(double time);

virtual void stop();
virtual void control_loop(double time);
std::string getName() const {return _name;}

robot_map& shareRobots(){return _robot_ptr;};
std::unique_ptr<mgnss::modules::Base> releaseController(){return std::move(_controller_ptr);}


virtual std::vector<std::string> readRobots(std::string config_file, std::string secondary_file, YAML::Node config, YAML::Node plugin_config);
virtual std::string _readRobot(std::string config_file, std::string secondary_file, YAML::Node config, YAML::Node plugin_config);

protected:
virtual void _setRate(double period){
        _controller_ptr->setRate(period);
}
virtual void _resetPrt(YAML::Node config) = 0;

virtual void _initCallbacks(YAML::Node config) = 0;

virtual void _initCallbacks(YAML::Node config, mwoibn::communication_modules::Shared& share){}

std::unique_ptr<mgnss::modules::Base> _controller_ptr;
std::map<std::string, std::shared_ptr<mwoibn::robot_class::Robot> > _robot_ptr;
std::shared_ptr<mwoibn::common::Logger> _logger_ptr;

virtual std::string _loadConfig(YAML::Node& config, YAML::Node& plugin_config);
virtual void _initModule(YAML::Node config, YAML::Node plugin_config);
virtual void _initModule(YAML::Node config, YAML::Node plugin_config, mwoibn::communication_modules::Shared& share);

virtual void _loadRobot(std::string config_file, std::string secondary_file, YAML::Node config, YAML::Node plugin_config);
virtual std::string _checkConfig(YAML::Node plugin_config, std::string config_file);


// virtual void _init();

std::shared_ptr<ros::NodeHandle> _n;
std::vector<ros::Subscriber> _sub;
std::vector<ros::ServiceServer> _srv;

bool _initialized = false, _valid = false, _rate = false;
std::string _name = "";
double _start;

};
}
}
#endif // RT_MY_TEST_H
