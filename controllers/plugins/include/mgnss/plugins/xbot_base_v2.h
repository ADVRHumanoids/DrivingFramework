#ifndef __MGNSS_PLUGINS_XBOT_BASE_UNIFY_H
#define __MGNSS_PLUGINS_XBOT_BASE_UNIFY_H

#include <XCM/XBotControlPlugin.h>
#include <mwoibn/robot_class/robot_xbot_rt.h>
#include "mgnss/modules/base.h"
#include "mgnss/plugins/generator.h"
#include <mwoibn/common/xbot_logger.h>

namespace mgnss::plugins
{
//! Wrapper over the ros publisher to provide a common interface in ROS and XBotCore middlewares. XBotCore implementation.
class XBotPublisher{

public:
  XBotPublisher(XBot::RosUtils::RosHandle::Ptr node): _node(node){
  }
  ~XBotPublisher(){}

  template<typename Message>
  /* Function to copy the publisher */
  void reset(const XBotPublisher& other){
      _pub = _pub;
  }

  /* Function to move the publisher */
  template<typename Message>
  void reset(XBotPublisher&& other){
    _pub = _pub;
  }

  //! Publish the message
  template<typename Message>
  void publish(const Message& msg ){
    _pub->pushToQueue(msg);
    _pub->popAndPublish();
  }

  //! Advertise the publisher
  template<typename Message>
 void advertise(const std::string& topic){
   _pub = _node->template advertise<Message>(topic, 1);
 }

protected:
  XBot::RosUtils::RosHandle::Ptr _node;
  XBot::RosUtils::PublisherWrapper::Ptr _pub;
};
}


//! C macro to keep readable name to dynamically load plugins
#define MGNSS_REGISTER_XBOT_PLUGIN_(constructor) \
    REGISTER_XBOT_PLUGIN_((mgnss::plugins::XBotPlugin<constructor<XBot::RosUtils::SubscriberWrapper::Ptr, XBot::RosUtils::ServiceServerWrapper::Ptr, XBot::RosUtils::RosHandle, mgnss::plugins::XBotPublisher> >)) \


namespace mgnss
{
namespace plugins
{

//! A class that directly provides the XBotCore plugins. It contains the Generator specialized for ros.
/** It inherits from the XBotControlPlugin and provides a wrapper over then Generator to comply with the XBotCore plugins interface
 *
 */
class XbotBaseUnify : public XBot::XBotControlPlugin
{

  // convinient variable that defines the specialization of the generator template
  typedef  mgnss::plugins::Generator<XBot::RosUtils::SubscriberWrapper::Ptr, XBot::RosUtils::ServiceServerWrapper::Ptr, XBot::RosUtils::RosHandle, XBotPublisher> XbotGenerator_;
  typedef  std::map<std::string, std::shared_ptr<mwoibn::robot_class::RobotXBotRT>> robot_map;


public:
  XbotBaseUnify(){
  }

  //! establish a connection with ROS
  void connect(XBot::Handle::Ptr handle)
  {
          _plugin_ptr->n = handle->getRosHandle();
  }

  //! creates the Generator plugin, robot model and logger
virtual bool init_control_plugin(XBot::Handle::Ptr handle);
  //! creates the Generator plugin, robot model and logger  in the shared space
virtual bool init_control_plugin(XBot::Handle::Ptr handle, robot_map& share_robots, std::shared_ptr<mwoibn::common::Logger>& logger_ptr, std::shared_ptr<XBot::RosUtils::RosHandle> n, mwoibn::communication_modules::Shared& share, std::string name);

  //! executed on the plugin shut down. It shut downs the modules, save the logger data to file, and closes the log file
virtual bool close(){_plugin_ptr->close();}

  //! executed when the plugin is turned on
  /** Checks the validity of all the feedbacks and sets initial conditions */
virtual void on_start(double time){_plugin_ptr->start(time);}

  //! executed when the plugin has been stoped
virtual void on_stop(double time){_plugin_ptr->stop();}

  // returns the reference to the instance of the specialized Generator template
XbotGenerator_& plugin(){return *_plugin_ptr;}

protected:

  std::unique_ptr<XbotGenerator_> _plugin_ptr; // instance of the Generator template specialized for XBotCore

  //! read config file from the xbotcore config file
  std::string _configFile(XBot::Handle::Ptr handle);

  //! executed at each control step, updates the module
  virtual void control_loop(double time, double period){
      _plugin_ptr->control_loop(time);}


};

//! Template over the XbotBaseUnify to dynamically load the requested plugin
template<typename Plugin>
class XBotPlugin: public XbotBaseUnify{
public:
    XBotPlugin(): XbotBaseUnify(){
        _plugin_ptr.reset(new Plugin());
    }
};
}
}
#endif // __MGNSS_PLUGINS_XBOT_BASE_UNIFY_H
