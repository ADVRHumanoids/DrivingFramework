#ifndef __MGNSS_XBOT_PLUGINS_JOINT_STATES_H
#define __MGNSS_XBOT_PLUGINS_JOINT_STATES_H

//#include <XCM/XBotControlPlugin.h>
//#include <mwoibn/robot_class/robot_xbot_rt.h>
#include "mgnss/controllers/joint_states.h"
#include "mgnss/ros_callbacks/joint_states.h"

#include "mgnss/plugins/xbot_base.h"

namespace mgnss
{
namespace xbot_plugins
{
class JointStates : public mgnss::plugins::XbotBase
{

protected:
virtual void _resetPrt(YAML::Node config)
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
virtual std::string _setName(){
        return "joint_states";
}


void control_loop(double time, double period)
{

        _valid = _robot_ptr->get();

        if (!_valid)
                return;

        if (!_initialized)
        {

                if(!_rate) {
                        _setRate(period); // here I may need a controller method
                        _rate = true;
                }
                if(_valid) {
                        _valid = _robot_ptr->feedbacks.reset();
                        _controller_ptr->init();
                }
                if(_rate && _valid)
                        _initialized = true;
        }

        _controller_ptr->update();

        //    _begin = std::chrono::high_resolution_clock::now();

        _controller_ptr->send();

        /*
            _end = std::chrono::high_resolution_clock::now();

            _logger_ptr->addField("update", std::chrono::duration_cast<std::chrono::microseconds>((_end-_begin)).count());
            _controller_ptr->log(*_logger_ptr.get(), time-_start);
         */
        //   std::cout <<  _robot_ptr->commad.velocity.get.transpose() << std::endl;

}


private:
// std::unique_ptr<mgnss::controllers::JointStates> _controller_ptr;
// std::unique_ptr<mwoibn::robot_class::Robot> _robot_ptr;
XBot::RosUtils::ServiceServerWrapper::Ptr _srv_rt;
// bool _initialized = false, _valid = false, _rate = false;
};
}
}
#endif // RT_MY_TEST_H
