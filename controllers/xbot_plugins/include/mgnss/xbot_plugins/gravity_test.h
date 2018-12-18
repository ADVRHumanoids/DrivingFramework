#ifndef __MGNSS_XBOT_PLUGINS_GRAVITY_TEST_H
#define __MGNSS_XBOT_PLUGINS_GRAVITY_TEST_H

#include <XCM/XBotControlPlugin.h>
#include <mwoibn/robot_class/robot_xbot_rt.h>

#include <mwoibn/hierarchical_control/controllers/default.h>
#include <mwoibn/hierarchical_control/tasks/constraints_task.h>
#include <mwoibn/hierarchical_control/tasks/cartesian_selective_task.h>
#include <mwoibn/hierarchical_control/tasks/orientation_selective_task.h>

namespace mgnss
{
namespace xbot_plugins
{
class GravityTest : public XBot::XBotControlPlugin
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
std::unique_ptr<mwoibn::robot_class::Robot> _robot_ptr;

std::unique_ptr<mwoibn::hierarchical_control::tasks::Constraints>
_constraints_ptr;
std::unique_ptr<mwoibn::hierarchical_control::tasks::CartesianSelective>
_pelvis_hight_ptr;
std::unique_ptr<mwoibn::hierarchical_control::tasks::OrientationSelective>
_pelvis_orientation_ptr;
mwoibn::hierarchical_control::controllers::Default hierarchical_controller;

double _eps = 0.001;

double _start_time;
mwoibn::VectorN _command, _state;
mwoibn::Vector3 _com_ref;
Eigen::Matrix<double, 4, 1> _com_final;

XBot::SubscriberRT<Eigen::Matrix<double, 4, 1> > _sub_com_final;
bool _initialized = false, _valid = false;
void _init();
void _getReference();
};
}
}
#endif // RT_MY_TEST_H
