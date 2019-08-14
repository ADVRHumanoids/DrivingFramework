#ifndef __MGNSS_ROS_PLUGINS_ODOMETRY_H
#define __MGNSS_ROS_PLUGINS_ODOMETRY_H

#include "mgnss/plugins/ros_base.h"
#include "mgnss/state_estimation/odometry.h"

namespace mgnss
{
namespace nrt_software {
namespace plugins
{
class Odometry : public mgnss::plugins::RosBase
{

public:
Odometry() : mgnss::plugins::RosBase(){}

virtual ~Odometry(){
}


protected:

virtual void _resetPrt(YAML::Node config){
        _controller_ptr.reset(new mgnss::state_estimation::Odometry(*_robot_ptr.begin()->second, config));
}

virtual void _initCallbacks(YAML::Node config){
}

};
}
}
}
#endif // RT_MY_TEST_H
