#ifndef __MGNSS_ROS_PLUGINS_WHEELS_CONTROLLERS_H
#define __MGNSS_ROS_PLUGINS_WHEELS_CONTROLLERS_H

#include <mgnss/plugins/ros_base.h>

#include <mgnss/controllers/wheels_controller_extend.h>
#include <mgnss/controllers/wheeled_motion_event.h>
#include <mgnss/controllers/wheeled_motion_event_v3.h>
#include <mgnss/controllers/wheeled_motion_world.h>

#include <mgnss/ros_callbacks/wheels_controller_extend.h>

namespace mgnss
{
namespace nrt_software{
namespace plugins
{


class WheelsControllerExtend : public mgnss::plugins::RosBase
{

public:
  WheelsControllerExtend(int argc, char** argv): mgnss::plugins::RosBase(argc, argv){
    _init(argc, argv);
  }

  bool init(){
    mgnss::plugins::RosBase::init();
    _support.noalias() = get().getSupportReference();
  }

  void start(double time)
  {
    mgnss::plugins::RosBase::start(time);
    if (_valid)
      _support.noalias() = get().getSupportReference();
  }

  mgnss::controllers::WheelsControllerExtend& get(){return static_cast<mgnss::controllers::WheelsControllerExtend&>(*_controller_ptr);}
  mwoibn::robot_class::Robot& robot(){return *_robot_ptr;}

  ~WheelsControllerExtend(){}


protected:
  ros::ServiceServer _srv_rt;
  ros::Subscriber _sub_rt;
  mwoibn::VectorN _support;
  virtual std::string _setName(){return "wheeled_motion";}

//  virtual void _resetPrt(YAML::Node config){
//    _controller_ptr.reset(new mgnss::controllers::WheelsControllerExtend(*_robot_ptr, config));
//  }

  virtual void _initCallbacks(){
    _srv_rt = _n->advertiseService<custom_services::updatePDGains::Request,
        custom_services::updatePDGains::Response>("wheels_command", boost::bind(&mgnss::ros_callbacks::wheels_controller_extend::eventsHandler,
                                                                                     _1, _2, static_cast<mgnss::controllers::WheelsControllerExtend*>(_controller_ptr.get())));
    _sub_rt = _n->subscribe<custom_messages::CustomCmnd>("wheels_support", 1, boost::bind(&mgnss::ros_callbacks::wheels_controller_extend::supportHandler,_1, &_support, static_cast<mgnss::controllers::WheelsControllerExtend*>(_controller_ptr.get()))); }

};

class WheeledMotionEvent : public WheelsControllerExtend
{
  public:
    WheeledMotionEvent(int argc, char** argv): WheelsControllerExtend(argc, argv){}
    ~WheeledMotionEvent(){}

  protected:
      virtual void _resetPrt(YAML::Node config){
        _controller_ptr.reset(new mgnss::controllers::WheeledMotionEvent(*_robot_ptr, config));
      }
};

class WheeledMotionEvent3 : public WheelsControllerExtend{
  public:
    WheeledMotionEvent3(int argc, char** argv): WheelsControllerExtend(argc, argv){}
    ~WheeledMotionEvent3(){}

  protected:
      virtual void _resetPrt(YAML::Node config){
        _controller_ptr.reset(new mgnss::controllers::WheeledMotionEvent3(*_robot_ptr, config));
      }
};

class WheeledMotionWorld : public WheelsControllerExtend{
  public:
    WheeledMotionWorld(int argc, char** argv): WheelsControllerExtend(argc, argv){}
    ~WheeledMotionWorld(){}

  protected:
      virtual void _resetPrt(YAML::Node config){
        _controller_ptr.reset(new mgnss::controllers::WheeledMotionWorld(*_robot_ptr, config));
      }
};

}
}
}
#endif // RT_MY_TEST_H
