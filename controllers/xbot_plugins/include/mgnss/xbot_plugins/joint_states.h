#ifndef __MGNSS_XBOT_PLUGINS_JOINT_STATES_H
#define __MGNSS_XBOT_PLUGINS_JOINT_STATES_H

//#include <XCM/XBotControlPlugin.h>
//#include <mwoibn/robot_class/robot_xbot_rt.h>
#include <mgnss/controllers/joint_states.h>
#include <mgnss/ros_callbacks/joint_states.h>

#include <mgnss/plugins/xbot_base.h>

namespace mgnss
{
namespace xbot_plugins
{
class JointStates : public mgnss::plugins::XbotBase
{

protected:
  virtual void _resetPrt(std::string config_file)
  {
    _controller_ptr.reset(new mgnss::controllers::JointStates(*_robot_ptr));
  }

  virtual void _initCallbacks(XBot::Handle::Ptr handle)
  {
    _srv_rt =
        handle->getRosHandle()
            ->advertiseService<custom_services::jointStateCmnd::Request,
                               custom_services::jointStateCmnd::Response>(
                "trajectory",
                boost::bind(&mgnss::ros_callbacks::joint_states::referenceHandler,
                            _1, _2, static_cast<mgnss::controllers::JointStates*>(_controller_ptr.get())));
  }
  virtual std::string _setName(){return "joint_states";}

private:
  // std::unique_ptr<mgnss::controllers::JointStates> _controller_ptr;
  // std::unique_ptr<mwoibn::robot_class::Robot> _robot_ptr;
  XBot::RosUtils::ServiceServerWrapper::Ptr _srv_rt;
  // bool _initialized = false, _valid = false, _rate = false;
};
}
}
#endif // RT_MY_TEST_H
