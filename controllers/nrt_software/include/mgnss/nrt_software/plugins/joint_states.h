#ifndef __MGNSS_ROS_PLUGINS_JOINT_STATES_H
#define __MGNSS_ROS_PLUGINS_JOINT_STATES_H

//#include "mgnss/plugins/ros_base.h"
#include "mgnss/plugins/generator.h"
#include "mgnss/ros_callbacks/joint_states.h"
#include "mgnss/controllers/joint_states.h"
#include <custom_services/jointStateCmnd.h>

namespace mgnss
{
namespace nrt_software {
namespace plugins
{
template<typename Subscriber, typename Service, typename Node>
class JointStates : public mgnss::plugins::Generator<Subscriber, Service, Node>
{
  typedef mgnss::plugins::Generator<Subscriber, Service, Node> Generator_;

public:
// JointStates(int argc, char** argv) : mgnss::plugins::RosBase(argc, argv, "joint_states"){}
JointStates() : Generator_("joint_states"){ }

virtual ~JointStates(){}

protected:

virtual void _resetPrt(YAML::Node config){
        Generator_::controller_ptr.reset(new mgnss::controllers::JointStates(*Generator_::_robot_ptr.begin()->second));
}

virtual void _initCallbacks(YAML::Node config){
        Generator_::_srv.push_back(Generator_::n->template advertiseService<custom_services::jointStateCmnd::Request,
                                   custom_services::jointStateCmnd::Response>( "trajectory",
                                    boost::bind(&mgnss::ros_callbacks::joint_states::referenceHandler,
                                   _1, _2, static_cast<mgnss::controllers::JointStates*>(Generator_::controller_ptr.get()))));

}


};


}
}
}

#endif // RT_MY_TEST_H
