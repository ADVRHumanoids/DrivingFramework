#ifndef __MGNSS_ROS_PLUGINS__ACCELERATION_TEST_H
#define __MGNSS_ROS_PLUGINS__ACCELERATION_TEST_H

//#include "mgnss/plugins/ros_base.h"
#include "mgnss/plugins/generator.h"
#include "mgnss/state_estimation/acceleration_test.h"

namespace mgnss
{
namespace nrt_software {
namespace plugins
{

  template<typename Subscriber, typename Service, typename Node, typename Publisher>
  class AccelerationTest : public mgnss::plugins::Generator<Subscriber, Service, Node, Publisher>
  {
    typedef mgnss::plugins::Generator<Subscriber, Service, Node, Publisher> Generator_;

    public:
    AccelerationTest() : Generator_("acceleration_test"){}

    virtual ~AccelerationTest(){
    }


protected:

virtual void _resetPrt(YAML::Node config){
        Generator_::controller_ptr.reset(new mgnss::state_estimation::AccelerationTest(*Generator_::_robot_ptr.begin()->second, config));
}

virtual void _initCallbacks(YAML::Node config){
}

};
}
}
}
#endif // RT_MY_TEST_H
