#ifndef __MGNSS_XBOT_PLUGINS_RT_COMBINED_CONTROLLER_H
#define __MGNSS_XBOT_PLUGINS_RT_COMBINED_CONTROLLER_H

#include <XCM/XBotControlPlugin.h>

#include <mwoibn/robot_class/robot_xbot_rt.h>
#include "mgnss/controllers/wheeled_motion.h"
#include "mgnss/controllers/online_centralized_controller.h"

namespace mgnss
{
namespace xbot_plugins
{
class CombinedController : public XBot::XBotControlPlugin
{

public:
virtual bool init_control_plugin(std::string path_to_config_file,
                                 XBot::SharedMemory::Ptr shared_memory,
                                 XBot::RobotInterface::Ptr robot);
virtual bool close();
virtual void on_start(double time);
virtual void on_stop(double time);

protected:
virtual void control_loop(double time);

private:
void _readReference();
bool _initialized = false, _valid = false;
std::unique_ptr<mwoibn::robot_class::Robot> _robot_ptr;
std::unique_ptr<mwoibn::WheeledMotion> _wheels_controller_ptr;
std::unique_ptr<mgnss::controllers::OnlineCentralizedController> _centralized_controller_ptr;

Eigen::Matrix<double, 12, 1> _references;
XBot::SubscriberRT<Eigen::Matrix<double, 13, 1> > _sub_references;

};
}
}
#endif // RT_MY_TEST_H
