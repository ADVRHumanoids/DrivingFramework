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
Odometry(int argc, char** argv) : mgnss::plugins::RosBase(argc, argv){
        _init(argc, argv);
}

//  mgnss::modules::Base& get(){return *_controller_ptr;}
//  mwoibn::robot_class::Robot& robot(){return *_robot_ptr;}

~Odometry(){
}


protected:
virtual std::string _setName(){
        return "odometry";
}

virtual void _resetPrt(YAML::Node config){
        _controller_ptr.reset(new mgnss::state_estimation::Odometry(*_robot_ptr, config));
}

virtual void _initCallbacks(){
}

};
}
}
}
#endif // RT_MY_TEST_H
