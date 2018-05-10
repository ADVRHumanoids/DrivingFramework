#ifndef __MGNSS_ROS_PLUGINS_EVENT_H
#define __MGNSS_ROS_PLUGINS_EVENT_H

#include <mgnss/plugins/ros_base.h>
#include <mgnss/controllers/wheeled_motion_world.h>
#include <mgnss/ros_callbacks/wheeled_motion_world.h>

namespace mgnss
{
namespace nrt_software{
namespace plugins
{
class WheeledMotionWorld : public mgnss::plugins::RosBase
{

public:
  WheeledMotionWorld(int argc, char** argv): mgnss::plugins::RosBase(argc, argv){
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

  mgnss::controllers::WheeledMotionWorld& get(){return static_cast<mgnss::controllers::WheeledMotionWorld&>(*_controller_ptr);}
  mwoibn::robot_class::Robot& robot(){return *_robot_ptr;}

  ~WheeledMotionWorld(){}


protected:
  ros::ServiceServer _srv_rt;
  ros::Subscriber _sub_rt;
  mwoibn::VectorN _support;
  virtual std::string _setName(){return "wheeled_motion";}

  virtual void _resetPrt(YAML::Node config){
    _controller_ptr.reset(new mgnss::controllers::WheeledMotionWorld(*_robot_ptr, config));
  }

  virtual void _initCallbacks(){
    _srv_rt = _n->advertiseService<custom_services::updatePDGains::Request,
        custom_services::updatePDGains::Response>("wheels_command", boost::bind(&mgnss::ros_callbacks::wheeled_motion_world::evenstHandler,
                                                                                     _1, _2, static_cast<mgnss::controllers::WheeledMotionWorld*>(_controller_ptr.get())));
//    _sub_rt = _n->subscribe<custom_messages::CustomCmnd>("wheels_support", 1, &mgnss::xbot_plugins::WheelsV2::supportHandler, this);
    _sub_rt = _n->subscribe<custom_messages::CustomCmnd>("wheels_support", 1, boost::bind(&mgnss::ros_callbacks::wheeled_motion_world::supportHandler,_1, &_support, static_cast<mgnss::controllers::WheeledMotionWorld*>(_controller_ptr.get()))); }

};
}
}
}
#endif // RT_MY_TEST_H
