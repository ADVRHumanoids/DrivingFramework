#ifndef __MGNSS_PLUGINS_ROS_SHARED_H
#define __MGNSS_PLUGINS_ROS_SHARED_H

#include "mgnss/modules/base.h"
#include "mgnss/plugins/ros_factory.h"
#include <mwoibn/common/ros_logger.h>
#include <mwoibn/communication_modules/shared.h>
#include <ros/ros.h>

namespace mgnss
{
namespace plugins
{
class RosShared
{

public:
// RosShared(int argc, char** argv): _name("shared"){
//   _init(argc, argv);
// }

RosShared() {
  // _init();
}

virtual ~RosShared(){
}

virtual void connect(int argc, char** argv, std::string name);
virtual void connect(std::string name);

virtual bool init();

virtual bool close();

virtual void start(double time);

virtual void stop();
virtual void control_loop(double time);

protected:
virtual void _setRate(double period){
    for(auto& controller: _controller_ptrs)
        controller->setRate(period);
}


std::vector<std::unique_ptr<mgnss::modules::Base>> _controller_ptrs;
std::map<std::string, std::shared_ptr<mwoibn::robot_class::Robot>> _robot_ptr;
std::shared_ptr<mwoibn::common::Logger> _logger_ptr;
mwoibn::communication_modules::Shared _shared;

virtual std::string _loadConfig(YAML::Node& config, YAML::Node& plugin_config);
virtual void _initModules(YAML::Node plugin_config);
virtual void _checkConfig(YAML::Node plugin_config, std::string config_file);

//
// virtual void _init(int argc, char** argv);
// virtual void _init();

std::shared_ptr<ros::NodeHandle> _n;

bool _initialized = false, _valid = false, _rate = false;
std::string _name = "";
double _start;

virtual void _init(double time);
virtual void _resetUpdates();

};
}
}
#endif // RT_MY_TEST_H
