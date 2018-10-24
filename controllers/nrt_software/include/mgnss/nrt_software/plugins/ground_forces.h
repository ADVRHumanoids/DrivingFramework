#ifndef __MGNSS_ROS_PLUGINS__GROUND_FORCES_H
#define __MGNSS_ROS_PLUGINS__GROUND_FORCES_H

#include "mgnss/plugins/ros_base.h"
#include "mgnss/state_estimation/ground_forces.h"

namespace mgnss
{
namespace nrt_software {
namespace plugins
{
class GroundForces : public mgnss::plugins::RosBase
{

public:
GroundForces(int argc, char** argv) : mgnss::plugins::RosBase(argc, argv){
        _init(argc, argv);
}

//  mgnss::modules::Base& get(){return *_controller_ptr;}
//  mwoibn::robot_class::Robot& robot(){return *_robot_ptr;}

virtual ~GroundForces(){
}


protected:
virtual std::string _setName(){
        return "ground_forces";
}

virtual void _resetPrt(YAML::Node config){
        _controller_ptr.reset(new mgnss::state_estimation::GroundForces(*_robot_ptr, config));
}

virtual void _initCallbacks(){
}

};
}
}
}
#endif // RT_MY_TEST_H
