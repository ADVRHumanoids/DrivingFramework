#ifndef __MGNSS_ROS_PLUGINS_ODOMETRY_2_H
#define __MGNSS_ROS_PLUGINS_ODOMETRY_2_H

#include "mgnss/plugins/ros_base.h"
#include "mgnss/state_estimation/odometry_v2.h"

namespace mgnss
{
namespace nrt_software {
namespace plugins
{
class Odometry2 : public mgnss::plugins::RosBase
{

public:
Odometry2() : mgnss::plugins::RosBase() {}

virtual ~Odometry2(){
}


protected:

virtual void _resetPrt(YAML::Node config){
        _controller_ptr.reset(new mgnss::state_estimation::OdometryV2(*_robot_ptr.begin()->second, config));
}

virtual void _initCallbacks(YAML::Node config){
}

};
}
}
}
#endif // RT_MY_TEST_H
