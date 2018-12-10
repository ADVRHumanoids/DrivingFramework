#ifndef __MGNSS_ROS_PLUGINS_WHEELS_CONTROLLERS_H
#define __MGNSS_ROS_PLUGINS_WHEELS_CONTROLLERS_H

#include "mgnss/plugins/ros_base.h"

#include "mgnss/controllers/wheels_controller_extend.h"

#include "mgnss/controllers/wheeled_motion_event_v3.h"
#include "mgnss/controllers/wheels_zmp.h"

#include "mgnss/controllers/wheels_reactif.h"
#include "mgnss/controllers/wheeled_motion_world.h"
//#include "mgnss/controllers/wheeled_motion_actions.h"
#include "mgnss/controllers/wheeled_motion_merge_v1.h"

#include "mgnss/ros_callbacks/wheels_zmp.h"

#include "mgnss/ros_callbacks/wheels_controller_extend.h"
#include "mgnss/ros_callbacks/wheels_controller_events.h"
#include "mgnss/ros_callbacks/wheels_controller_actions.h"
#include "mgnss/ros_callbacks/wheels_controller_merge_v1.h"


namespace mgnss {
namespace nrt_software {
namespace plugins {

class WheelsControllerExtend : public mgnss::plugins::RosBase
{

public:
// WheelsControllerExtend(int argc, char** argv) : mgnss::plugins::RosBase(argc, argv, "wheeled_motion"){
// }
WheelsControllerExtend(std::string name = "") : mgnss::plugins::RosBase(){
}

bool init(){
        mgnss::plugins::RosBase::init();
}

mgnss::controllers::WheelsControllerExtend& get(){
        return static_cast<mgnss::controllers::WheelsControllerExtend&>(*_controller_ptr);
}
mwoibn::robot_class::Robot& robot(){
        return *_robot_ptr.begin()->second;
}

virtual ~WheelsControllerExtend(){
}

protected:
    virtual void _initCallbacks(YAML::Node config){
        _srv.push_back(_n->advertiseService<custom_services::updatePDGains::Request,
                                       custom_services::updatePDGains::Response>("wheels_command", boost::bind(&mgnss::ros_callbacks::wheels_controller_extend::eventsHandler,_1, _2, static_cast<mgnss::controllers::WheelsControllerExtend*>(_controller_ptr.get()))));

        _sub.push_back(_n->subscribe<custom_messages::CustomCmnd>("wheels_support", 1, boost::bind(&mgnss::ros_callbacks::wheels_controller_extend::supportHandler,_1, static_cast<mgnss::controllers::WheelsControllerExtend*>(_controller_ptr.get()))));
}

};

class WheeledMotionEvent3 : public WheelsControllerExtend {
public:
// WheeledMotionEvent3(int argc, char** argv) : WheelsControllerExtend(argc, argv){
// }
WheeledMotionEvent3() : WheelsControllerExtend(){}
virtual ~WheeledMotionEvent3(){
}

protected:
virtual void _resetPrt(YAML::Node config){
        _controller_ptr.reset(new mgnss::controllers::WheeledMotionEvent3(*_robot_ptr.begin()->second, config));
}

virtual void _initCallbacks(YAML::Node config){
        _srv.push_back(_n->advertiseService<custom_services::updatePDGains::Request,
                                       custom_services::updatePDGains::Response>("wheels_command", boost::bind(&mgnss::ros_callbacks::wheels_controller_events::eventsHandler,  _1, _2, static_cast<mgnss::controllers::WheelsControllerExtend*>(_controller_ptr.get()))));

        _sub.push_back(_n->subscribe<custom_messages::CustomCmnd>("wheels_support", 1, boost::bind(&mgnss::ros_callbacks::wheels_controller_events::supportHandler,_1, static_cast<mgnss::controllers::WheelsControllerExtend*>(_controller_ptr.get()))));

        _sub.push_back(_n->subscribe<custom_messages::StateMsg>("wheels_state", 1, boost::bind(&mgnss::ros_callbacks::wheels_controller_events::stateHandler,_1, static_cast<mgnss::controllers::WheelsControllerExtend*>(_controller_ptr.get()))));

}

};

class WheelsReactif : public WheeledMotionEvent3 {
public:

WheelsReactif() : WheeledMotionEvent3(){}
virtual ~WheelsReactif(){
}

protected:
virtual void _resetPrt(YAML::Node config){
        _controller_ptr.reset(new mgnss::controllers::WheelsReactif(*_robot_ptr.begin()->second, config));
}

};

class WheeledMotionWorld : public WheelsControllerExtend {
public:
WheeledMotionWorld() : WheelsControllerExtend(){}
virtual ~WheeledMotionWorld(){
}

protected:
virtual void _resetPrt(YAML::Node config){
        _controller_ptr.reset(new mgnss::controllers::WheeledMotionWorld(*_robot_ptr.begin()->second, config));
}
};

class WheelsZMP : public WheelsControllerExtend {
public:

  WheelsZMP() : WheelsControllerExtend(){}
  virtual ~WheelsZMP(){
  }

protected:
  virtual void _resetPrt(YAML::Node config){
          _controller_ptr.reset(new mgnss::controllers::WheelsZMP(*_robot_ptr.begin()->second, config));
  }
};


class WheeledMotionMergeV1 : public WheeledMotionEvent3 {
public:

WheeledMotionMergeV1() : WheeledMotionEvent3(){}
virtual ~WheeledMotionMergeV1(){
}

protected:
virtual void _resetPrt(YAML::Node config){
        _controller_ptr.reset(new mgnss::controllers::WheeledMotionMergeV1(*_robot_ptr.begin()->second, config));
}
virtual void _initCallbacks(YAML::Node config){
        _srv.push_back(_n->advertiseService<custom_services::updatePDGains::Request,
                                       custom_services::updatePDGains::Response>("wheels_command", boost::bind(&mgnss::ros_callbacks::wheels_controller_merge::eventsHandler,_1, _2, static_cast<mgnss::controllers::WheeledMotionMergeV1*>(_controller_ptr.get()))));

        _sub.push_back(_n->subscribe<custom_messages::CustomCmnd>("wheels_support", 1, boost::bind(&mgnss::ros_callbacks::wheels_controller_extend::supportHandler,_1, static_cast<mgnss::controllers::WheelsControllerExtend*>(_controller_ptr.get()))));

        _sub.push_back(_n->subscribe<custom_messages::StateMsg>("wheels_state", 1, boost::bind(&mgnss::ros_callbacks::wheels_controller_events::stateHandler,_1, static_cast<mgnss::controllers::WheelsControllerExtend*>(_controller_ptr.get()))));
}

};

}
}
}
#endif // RT_MY_TEST_H
