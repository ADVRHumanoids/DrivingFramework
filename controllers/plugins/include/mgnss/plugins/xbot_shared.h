#ifndef __MGNSS_PLUGINS_XBOT_SHARED_H
#define __MGNSS_PLUGINS_XBOT_SHARED_H

#include "mgnss/modules/base.h"
#include "mgnss/plugins/xbot_factory.h"
#include <mwoibn/common/xbot_logger.h>
#include <mgnss/plugins/xbot_base_v2.h>

#include <mwoibn/communication_modules/shared.h>
#include <ros/ros.h>

namespace mgnss::plugins
{
class XbotShared: public XBot::XBotControlPlugin
{

public:
// XbotShared(int argc, char** argv): _name("shared"){
//   _init(argc, argv);
// }

XbotShared() {
  // _init();
}

virtual ~XbotShared(){
}

void connect(XBot::Handle::Ptr handle);

virtual bool init_control_plugin(XBot::Handle::Ptr handle);


virtual bool close();

virtual void on_start(double time);

virtual void on_stop(double time);

virtual void control_loop(double time);

protected:
virtual void _setRate(double period){
    for(auto& controller: _controller_ptrs)
        controller->setRate(period);
}

virtual void control_loop(double time, double period){
    control_loop(time);}

std::vector<std::unique_ptr<mgnss::modules::Base>> _controller_ptrs;

std::map<std::string, std::shared_ptr<mwoibn::robot_class::RobotXBotRT>> _robot_ptr;
std::shared_ptr<mwoibn::common::Logger> _logger_ptr;
mwoibn::communication_modules::Shared _shared;

virtual std::string _loadConfig(XBot::Handle::Ptr handle, YAML::Node& config, YAML::Node& plugin_config);
virtual void _initModules(XBot::Handle::Ptr handle, YAML::Node plugin_config);
virtual void _checkConfig(YAML::Node plugin_config, std::string config_file);


std::shared_ptr<XBot::RosUtils::RosHandle> _n;

bool _initialized = false, _valid = false, _rate = false;
std::string _name = "";
double _start;

};
}
#endif // RT_MY_TEST_H
