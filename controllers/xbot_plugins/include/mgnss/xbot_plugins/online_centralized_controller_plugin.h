#ifndef __MGNSS_XBOT_PLUGINS_ONLINE_CENTRALIZED_CONTROLLER_PLUGIN_H
#define __MGNSS_XBOT_PLUGINS_ONLINE_CENTRALIZED_CONTROLLER_PLUGIN_H

#include <XCM/XBotControlPlugin.h>
#include <mwoibn/robot_class/robot_xbot_rt.h>
#include "mgnss/controllers/online_centralized_controller.h"

namespace mgnss {

namespace xbot_plugins {

class OnlineCentralizedControllerPlugin : public XBot::XBotControlPlugin {


public:

virtual bool init_control_plugin(std::string path_to_config_file,
                                 XBot::SharedMemory::Ptr shared_memory,
                                 XBot::RobotInterface::Ptr robot);

virtual bool close();

virtual void on_start(double time);

virtual void on_stop(double time);

protected:

virtual void control_loop(double time, double period);

private:

//  double _start_time;

std::unique_ptr<mwoibn::robot_class::Robot> _robot_ptr;
std::unique_ptr<mwoibn::robot_class::Robot> _robot_ref_ptr;

std::unique_ptr<mgnss::controllers::OnlineCentralizedController> _controller_ptr;   // online set up

bool _motor_side = false;
bool _valid = false;


};
}
}
#endif // __MGNSS_RT_PLUGINS_RT_MY_TEST_H
