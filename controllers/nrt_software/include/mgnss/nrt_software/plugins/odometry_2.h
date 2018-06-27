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
Odometry2(int argc, char** argv) : mgnss::plugins::RosBase(argc, argv){
        _init(argc, argv);
}

//  mgnss::modules::Base& get(){return *_controller_ptr;}
//  mwoibn::robot_class::Robot& robot(){return *_robot_ptr;}

virtual ~Odometry2(){
}


protected:
virtual std::string _setName(){
        return "odometry";
}

virtual void _resetPrt(YAML::Node config){
        _controller_ptr.reset(new mgnss::state_estimation::OdometryV2(*_robot_ptr, config));
}

virtual void _initCallbacks(){
}

};
}
}
}
#endif // RT_MY_TEST_H
