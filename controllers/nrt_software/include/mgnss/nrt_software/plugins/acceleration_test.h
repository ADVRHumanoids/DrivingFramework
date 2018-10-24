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
AccelerationTest(int argc, char** argv) : mgnss::plugins::RosBase(argc, argv){
        _init(argc, argv);
}

//  mgnss::modules::Base& get(){return *_controller_ptr;}
//  mwoibn::robot_class::Robot& robot(){return *_robot_ptr;}

virtual ~AccelerationTest(){
}


protected:
virtual std::string _setName(){
        return "acceleration_test";
}

virtual void _resetPrt(YAML::Node config){
        _controller_ptr.reset(new mgnss::state_estimation::AccelerationTest(*_robot_ptr, config));
}

virtual void _initCallbacks(){
}

};
}
}
}
#endif // RT_MY_TEST_H
