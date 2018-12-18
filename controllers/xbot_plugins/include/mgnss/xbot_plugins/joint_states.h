#ifndef __MGNSS_XBOT_PLUGINS_JOINT_STATES_H
#define __MGNSS_XBOT_PLUGINS_JOINT_STATES_H

#include "mgnss/controllers/joint_states.h"
#include "mgnss/ros_callbacks/joint_states.h"

#include "mgnss/plugins/xbot_base.h"

namespace mgnss
{
namespace xbot_plugins
{
class JointStates : public mgnss::plugins::XbotBase
{
public:
  JointStates(int argc, char** argv) : mgnss::plugins::XbotBase(argc, argv, "joint_states"){
  }

  JointStates() : mgnss::plugins::XbotBase("joint_states"){}

  virtual ~JointStates(){}


protected:
virtual void _resetPrt(YAML::Node config)
{
        _controller_ptr.reset(new mgnss::controllers::JointStates(*_robot_ptr));
}

virtual void _initCallbacks(YAML::Node config)
{
        _srv_rt =  _n->advertiseService<custom_services::jointStateCmnd::Request,
                       custom_services::jointStateCmnd::Response>("trajectory",
                        boost::bind(&mgnss::ros_callbacks::joint_states::referenceHandler,
                                    _1, _2, static_cast<mgnss::controllers::JointStates*>(_controller_ptr.get())));
}



void control_loop(double time)
{

      _valid = _robot_ptr->get();

      if (!_valid)
          return;

      if (!_initialized)
      {
          _setRate(_robot_ptr->rate());
          _valid = _robot_ptr->feedbacks.reset();
          if(_valid) {
                  _controller_ptr->init();
                  _initialized = true;
          }
      }


      _controller_ptr->update();
      _controller_ptr->send();

}


private:
  XBot::RosUtils::ServiceServerWrapper::Ptr _srv_rt;

};
}
}

#endif // RT_MY_TEST_H
