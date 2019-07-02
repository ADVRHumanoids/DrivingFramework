#ifndef __MGNSS_ROS_PLUGINS_UPPER_BODY_IK_H
#define __MGNSS_ROS_PLUGINS_UPPER_BODY_IK_H

//#include "mgnss/plugins/ros_base.h"
#include "mgnss/plugins/generator.h"
#include "mgnss/ros_callbacks/upper_body_ik.h"
#include "mgnss/controllers/upper_body_IK.h"
#include <mgnss_utils/point.h>

namespace mgnss
{
namespace nrt_software {
namespace plugins
{
template<typename Subscriber, typename Service, typename Node>
class UpperBodyIK : public mgnss::plugins::Generator<Subscriber, Service, Node>
{
  typedef mgnss::plugins::Generator<Subscriber, Service, Node> Generator_;

public:
// JointStates(int argc, char** argv) : mgnss::plugins::RosBase(argc, argv, "joint_states"){}
UpperBodyIK() : Generator_("upper_body_ik"){ }

virtual ~UpperBodyIK(){}

protected:

virtual void _resetPrt(YAML::Node config){
        Generator_::controller_ptr.reset(new mgnss::controllers::UpperBodyIK(*Generator_::_robot_ptr.begin()->second, config));
}

virtual void _initCallbacks(YAML::Node config){
        Generator_::_srv.push_back(Generator_::n->template advertiseService<mgnss_utils::point::Request,
                                   mgnss_utils::point::Response>( "ub_reference",
                                    boost::bind(&mgnss::ros_callbacks::upper_body_ik::setReference,
                                   _1, _2, static_cast<mgnss::controllers::UpperBodyIK*>(Generator_::controller_ptr.get()))));

}


};


}
}
}

#endif // RT_MY_TEST_H
