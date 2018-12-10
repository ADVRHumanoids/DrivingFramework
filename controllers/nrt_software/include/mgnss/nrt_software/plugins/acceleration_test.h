#ifndef __MGNSS_ROS_PLUGINS__ACCELERATION_TEST_H
#define __MGNSS_ROS_PLUGINS__ACCELERATION_TEST_H

#include "mgnss/plugins/ros_base.h"
#include "mgnss/state_estimation/acceleration_test.h"

namespace mgnss
{
namespace nrt_software {
namespace plugins
{
class AccelerationTest : public mgnss::plugins::RosBase
{

public:
AccelerationTest() : mgnss::plugins::RosBase(){}

virtual ~AccelerationTest(){
}


protected:

virtual void _resetPrt(YAML::Node config){
        _controller_ptr.reset(new mgnss::state_estimation::AccelerationTest(*_robot_ptr.begin()->second, config));
}

virtual void _initCallbacks(YAML::Node config){
}

};
}
}
}
#endif // RT_MY_TEST_H
