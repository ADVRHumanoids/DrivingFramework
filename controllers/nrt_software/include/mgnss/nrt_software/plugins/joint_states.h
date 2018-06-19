#ifndef __MGNSS_ROS_PLUGINS_JOINT_STATES_H
#define __MGNSS_ROS_PLUGINS_JOINT_STATES_H

#include "mgnss/plugins/ros_base.h"
#include "mgnss/ros_callbacks/joint_states.h"
#include "mgnss/controllers/joint_states.h"

namespace mgnss
{
namespace nrt_software {
namespace plugins
{
class JointStates : public mgnss::plugins::RosBase
{

public:
JointStates(int argc, char** argv) : mgnss::plugins::RosBase(argc, argv){
        _init(argc, argv);
}

virtual ~JointStates(){
}

protected:
virtual std::string _setName(){
        return "joint_states";
}
virtual void _resetPrt(YAML::Node config){
        _controller_ptr.reset(new mgnss::controllers::JointStates(*_robot_ptr));
}

virtual void _initCallbacks(){
        _srv_rt =
                _n->advertiseService<custom_services::jointStateCmnd::Request,
                                     custom_services::jointStateCmnd::Response>(
                        "trajectory",
                        boost::bind(&mgnss::ros_callbacks::joint_states::referenceHandler,
                                    _1, _2, static_cast<mgnss::controllers::JointStates*>(_controller_ptr.get())));

}

//virtual void _init(int argc, char** argv);

ros::ServiceServer _srv_rt;
//double _start;

};
}
}
}
#endif // RT_MY_TEST_H
